import SwiftUI
import AVFoundation

@main
struct CatDoorCameraApp: App {
    var body: some Scene {
        WindowGroup { CameraView() }
    }
}

struct CameraView: View {
    @StateObject private var camera = CameraController()
    @Environment(\.scenePhase) private var scenePhase
    @AppStorage("piAddress") private var address = "http://pi3.local:8080"
    @AppStorage("pullMode") private var pullMode = false
    @AppStorage("cropX") private var x = 0.5
    @AppStorage("cropY") private var y = 0.5
    @AppStorage("cropSize") private var size = 1.0

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 18) {
                Text("Cat door camera").font(.largeTitle.bold())
                TextField("Pi address", text: $address)
                    .textContentType(.URL).keyboardType(.URL)
                    .textInputAutocapitalization(.never).disableAutocorrection(true)
                    .textFieldStyle(.roundedBorder).disabled(camera.running)
                Picker("Capture mode", selection: $pullMode) {
                    Text("Push · every 10s").tag(false)
                    Text("Pull · Pi requests").tag(true)
                }.pickerStyle(.segmented).disabled(camera.running)
                Text(pullMode ? "The Pi requests each photo. Tap Start to listen." : "The phone sends a photo every ten seconds.")
                    .font(.footnote)
                CameraPreview(session: camera.session)
                    .aspectRatio(3.0 / 4.0, contentMode: .fit)
                    .overlay {
                        GeometryReader { geometry in
                            Rectangle().stroke(.yellow, lineWidth: 2)
                                .frame(width: geometry.size.width * size, height: geometry.size.height * size)
                                .offset(x: geometry.size.width * x * (1 - size),
                                        y: geometry.size.height * y * (1 - size))
                        }
                    }
                    .background(.black)
                Group {
                    Text("Crop size: \(Int(size * 100))%")
                    Slider(value: $size, in: 0.2...1)
                    Text("Horizontal position")
                    Slider(value: $x, in: 0...1)
                    Text("Vertical position")
                    Slider(value: $y, in: 0...1)
                }.disabled(camera.running)
                Button(camera.running ? "Stop" : (pullMode ? "Start · wait for Pi" : "Start · every 10 seconds")) {
                    if camera.running { camera.stop() }
                    else { camera.start(address: address, x: x, y: y, size: size, pull: pullMode) }
                }.buttonStyle(.borderedProminent)
                    .disabled(!camera.cameraReady && !camera.running)
                Text(camera.status)
                Text("\(camera.uploaded) uploads this session").foregroundColor(.secondary)
                if let image = camera.lastImage {
                    Text("Last captured crop")
                    Image(uiImage: image).resizable().scaledToFit()
                }
                Text("The screen stays awake while this app is open. Keep the phone powered. Leaving the app stops capture; tap Start when you return. Stop to adjust the crop. Photos go only to your Pi and are not saved to Photos.")
                    .font(.footnote).foregroundColor(.secondary)
            }.padding()
        }
        .onAppear { camera.setForeground(true) }
        .onDisappear { camera.setForeground(false) }
        .onChange(of: scenePhase) { phase in
            if phase == .active { camera.setForeground(true) }
            else if phase == .background { camera.setForeground(false) }
        }
    }
}

/// Displays the full portrait camera frame with no preview-only cropping.
private struct CameraPreview: UIViewRepresentable {
    let session: AVCaptureSession

    /// Returns:
    ///     PreviewView: A view whose backing layer displays the camera session.
    func makeUIView(context: Context) -> PreviewView {
        let view = PreviewView()
        view.preview.session = session
        view.preview.videoGravity = .resizeAspect
        return view
    }

    /// Keep the preview aligned with the portrait-only photo output.
    func updateUIView(_ view: PreviewView, context: Context) {
        if let connection = view.preview.connection, connection.isVideoOrientationSupported {
            connection.videoOrientation = .portrait
        }
    }
}

private final class PreviewView: UIView {
    override class var layerClass: AnyClass { AVCaptureVideoPreviewLayer.self }
    var preview: AVCaptureVideoPreviewLayer { layer as! AVCaptureVideoPreviewLayer }
}
