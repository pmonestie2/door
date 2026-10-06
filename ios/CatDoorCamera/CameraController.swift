import AVFoundation
import SwiftUI
import UIKit

/// Captures and uploads one cropped image at a time while the app is active.
final class CameraController: NSObject, ObservableObject, AVCapturePhotoCaptureDelegate {
    @Published private(set) var running = false
    @Published private(set) var cameraReady = false
    @Published private(set) var status = "Ready"
    @Published private(set) var lastImage: UIImage?
    @Published private(set) var uploaded = 0

    let session = AVCaptureSession()
    private let queue = DispatchQueue(label: "CatDoorCamera.capture")
    private let output = AVCapturePhotoOutput()
    private var configured = false
    private var active = false
    private var foreground = false
    private var previewGeneration = 0
    private var busy = false
    private var generation = 0
    private var shotGeneration = 0
    private var timer: DispatchSourceTimer?
    private var destination: URL?
    private var commandURL: URL?
    private var commandTask: URLSessionDataTask?
    private var pullMode = false
    private var captureRequestID: String?
    private var crop = CGRect(x: 0, y: 0, width: 1, height: 1)
    private var capturedAt = Date()
    private var uploadTask: URLSessionDataTask?
    private let uploader: URLSession = {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.urlCache = nil
        configuration.httpCookieStorage = nil
        configuration.requestCachePolicy = .reloadIgnoringLocalCacheData
        return URLSession(configuration: configuration)
    }()

    /// Keep the screen awake and preview active only while the app is in the foreground.
    func setForeground(_ visible: Bool) {
        UIApplication.shared.isIdleTimerDisabled = visible
        if !visible {
            stop()
            cameraReady = false
        }
        queue.async {
            guard self.foreground != visible else { return }
            self.foreground = visible
            self.previewGeneration += 1
            let requestedGeneration = self.previewGeneration
            if !visible {
                self.session.stopRunning()
                return
            }
            self.publishStatus("Opening camera…")
            AVCaptureDevice.requestAccess(for: .video) { allowed in
                self.queue.async {
                    guard self.foreground, self.previewGeneration == requestedGeneration else { return }
                    guard allowed else {
                        self.fail("Camera permission denied. Enable Camera in Settings → Apps → Cat Door Camera.")
                        return
                    }
                    do {
                        try self.configure()
                        self.session.startRunning()
                        guard self.session.isRunning else { throw CameraError.unavailable }
                        DispatchQueue.main.async {
                            self.cameraReady = true
                            self.status = "Camera ready. Adjust the crop, then tap Start."
                        }
                    } catch {
                        self.fail(error.localizedDescription)
                    }
                }
            }
        }
    }

    /// Validate settings and start uploading from the already-running camera.
    func start(address: String, x: Double, y: Double, size: Double, pull: Bool) {
        guard let base = URL(string: address.trimmingCharacters(in: .whitespacesAndNewlines)),
              ["http", "https"].contains(base.scheme?.lowercased() ?? ""),
              base.host != nil, base.user == nil, base.password == nil,
              base.query == nil, base.fragment == nil,
              base.path.isEmpty || base.path == "/" else {
            status = "Enter the Pi address, for example http://raspberrypi.local:8080"
            return
        }
        guard cameraReady else {
            status = "Wait for the camera preview before starting uploads."
            return
        }
        running = true
        queue.async {
            self.generation += 1
            self.active = true
            self.destination = base.appendingPathComponent("camera")
            self.commandURL = base.appendingPathComponent("camera/command")
            self.pullMode = pull
            self.crop = CGRect(x: x * (1 - size), y: y * (1 - size), width: size, height: size)
            if pull {
                self.publishStatus("Pull mode: waiting for the Pi")
                self.waitForCommand()
            } else {
                let timer = DispatchSource.makeTimerSource(queue: self.queue)
                timer.schedule(deadline: .now() + 1, repeating: 10)
                timer.setEventHandler { [weak controller = self] in controller?.capture() }
                self.timer?.cancel()
                self.timer = timer
                timer.resume()
                self.publishStatus("Push mode: taking a photo every 10 seconds")
            }
        }
    }

    /// Stop periodic photos and uploads while leaving the preview available for cropping.
    func stop() {
        running = false
        queue.async {
            self.active = false
            self.generation += 1
            self.timer?.cancel()
            self.timer = nil
            self.commandTask?.cancel()
            self.commandTask = nil
            self.uploadTask?.cancel()
            self.uploadTask = nil
            self.publishStatus("Uploads stopped. Adjust the crop, then tap Start.")
        }
    }

    /// Hold an HTTP request open until the Pi asks for a photo; reconnect after idle waits.
    private func waitForCommand() {
        guard active, pullMode, !busy, commandTask == nil, let url = commandURL else { return }
        let currentGeneration = generation
        var request = URLRequest(url: url)
        request.timeoutInterval = 25
        commandTask = uploader.dataTask(with: request) { data, response, error in
            self.queue.async {
                guard self.active, self.generation == currentGeneration else { return }
                self.commandTask = nil
                let code = (response as? HTTPURLResponse)?.statusCode ?? 0
                if error == nil && code == 204 {
                    self.waitForCommand()
                } else if error == nil, code == 200, let data = data,
                          let command = try? JSONDecoder().decode(CaptureCommand.self, from: data),
                          command.command == "capture" {
                    self.capture(requestID: command.request_id)
                    if !self.busy { self.retryCommandWait() }
                } else {
                    self.publishStatus("Pull connection failed: \(error?.localizedDescription ?? "HTTP \(code)"). Retrying…")
                    self.retryCommandWait()
                }
            }
        }
        commandTask?.resume()
    }

    /// Retry command delivery without queuing captures or reconnecting in a tight loop.
    private func retryCommandWait() {
        let currentGeneration = generation
        queue.asyncAfter(deadline: .now() + 2) {
            guard self.generation == currentGeneration else { return }
            self.waitForCommand()
        }
    }

    /// Configure the back camera once; all session changes happen on the capture queue.
    private func configure() throws {
        guard !configured else { return }
        guard let camera = AVCaptureDevice.default(.builtInWideAngleCamera, for: .video, position: .back) else {
            throw CameraError.unavailable
        }
        let input = try AVCaptureDeviceInput(device: camera)
        session.beginConfiguration()
        defer { session.commitConfiguration() }
        session.sessionPreset = .photo
        guard session.canAddInput(input) else { throw CameraError.unavailable }
        session.addInput(input)
        guard session.canAddOutput(output) else {
            session.removeInput(input)
            throw CameraError.unavailable
        }
        session.addOutput(output)
        output.maxPhotoQualityPrioritization = .speed
        configured = true
    }

    /// Take a fresh photo unless a capture or upload is already in progress.
    private func capture(requestID: String? = nil) {
        guard active, !busy, session.isRunning, !session.isInterrupted else { return }
        busy = true
        captureRequestID = requestID
        shotGeneration = generation
        capturedAt = Date()
        if let connection = output.connection(with: .video), connection.isVideoOrientationSupported {
            connection.videoOrientation = .portrait
        }
        let settings = AVCapturePhotoSettings(format: [AVVideoCodecKey: AVVideoCodecType.jpeg])
        if #available(iOS 18.0, *), output.isShutterSoundSuppressionSupported {
            settings.isShutterSoundSuppressionEnabled = true
        }
        settings.flashMode = .off
        settings.photoQualityPrioritization = .speed
        output.capturePhoto(with: settings, delegate: self)
    }

    /// Prepare a cropped JPEG and upload it after the camera has finished processing.
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishProcessingPhoto photo: AVCapturePhoto, error: Error?) {
        let data = photo.fileDataRepresentation()
        queue.async {
            guard self.active, self.shotGeneration == self.generation else {
                self.busy = false
                self.waitForCommand()
                return
            }
            guard error == nil, let data = data, let image = UIImage(data: data),
                  let jpeg = self.croppedJPEG(image) else {
                self.busy = false
                self.publishStatus("Capture failed: \(error?.localizedDescription ?? "No image")")
                self.retryCommandWait()
                return
            }
            DispatchQueue.main.async { self.lastImage = UIImage(data: jpeg) }
            self.upload(jpeg)
        }
    }

    /// Release a failed capture so later timer ticks can try again.
    func photoOutput(_ output: AVCapturePhotoOutput, didFinishCaptureFor resolvedSettings: AVCaptureResolvedPhotoSettings, error: Error?) {
        guard let error = error else { return }
        queue.async {
            self.busy = false
            if self.active {
                self.publishStatus("Camera error: \(error.localizedDescription)")
                self.retryCommandWait()
            }
        }
    }

    /// Returns:
    ///     Data?: An upright cropped JPEG, at most 640 pixels on its longest side, or nil on failure.
    private func croppedJPEG(_ image: UIImage) -> Data? {
        let format = UIGraphicsImageRendererFormat()
        format.scale = 1
        let fullScale = min(1, 1280 / max(image.size.width, image.size.height))
        let fullSize = CGSize(width: image.size.width * fullScale, height: image.size.height * fullScale)
        let upright = UIGraphicsImageRenderer(size: fullSize, format: format).image { _ in
            image.draw(in: CGRect(origin: .zero, size: fullSize))
        }
        guard let pixels = upright.cgImage else { return nil }
        let rect = CGRect(x: crop.minX * CGFloat(pixels.width), y: crop.minY * CGFloat(pixels.height),
                          width: crop.width * CGFloat(pixels.width), height: crop.height * CGFloat(pixels.height)).integral
        guard let cropped = pixels.cropping(to: rect) else { return nil }
        let factor = min(1, 640 / CGFloat(max(cropped.width, cropped.height)))
        let size = CGSize(width: CGFloat(cropped.width) * factor, height: CGFloat(cropped.height) * factor)
        let result = UIGraphicsImageRenderer(size: size, format: format).image { _ in
            UIImage(cgImage: cropped).draw(in: CGRect(origin: .zero, size: size))
        }
        return result.jpegData(compressionQuality: 0.8)
    }

    /// Upload without queuing old frames; failures are retried with the next fresh capture.
    private func upload(_ jpeg: Data) {
        guard let destination = destination else { busy = false; return }
        let currentGeneration = generation
        var request = URLRequest(url: destination)
        request.httpMethod = "POST"
        request.timeoutInterval = 8
        request.setValue("image/jpeg", forHTTPHeaderField: "Content-Type")
        request.setValue(ISO8601DateFormatter().string(from: capturedAt), forHTTPHeaderField: "X-Captured-At")
        if let requestID = captureRequestID {
            request.setValue(requestID, forHTTPHeaderField: "X-Capture-Request-ID")
        }
        request.httpBody = jpeg
        uploadTask = uploader.dataTask(with: request) { _, response, error in
            self.queue.async {
                self.busy = false
                guard self.active, self.generation == currentGeneration else {
                    self.waitForCommand()
                    return
                }
                defer { self.waitForCommand() }
                let code = (response as? HTTPURLResponse)?.statusCode ?? 0
                if error == nil && code == 201 {
                    DispatchQueue.main.async {
                        self.uploaded += 1
                        self.status = "Uploaded at \(DateFormatter.localizedString(from: Date(), dateStyle: .none, timeStyle: .medium))"
                    }
                } else {
                    if let failure = error as? URLError, failure.code == .timedOut {
                        self.publishStatus("Pi timed out at \(destination.absoluteString). Check the address, Wi-Fi, and Local Network permission; open the Pi address in Safari.")
                    } else {
                        self.publishStatus("Upload failed: \(error?.localizedDescription ?? "HTTP \(code)"). Waiting for the next capture.")
                    }
                }
            }
        }
        uploadTask?.resume()
    }

    /// Publish status text on the UI thread.
    private func publishStatus(_ text: String) {
        DispatchQueue.main.async { self.status = text }
    }

    /// Stop a failed session and show the reason.
    private func fail(_ text: String) {
        active = false
        DispatchQueue.main.async {
            self.running = false
            self.cameraReady = false
            self.status = text
        }
    }
}

private enum CameraError: LocalizedError {
    case unavailable
    var errorDescription: String? { "The back camera is unavailable." }
}

private struct CaptureCommand: Decodable {
    let command: String
    let request_id: String
}
