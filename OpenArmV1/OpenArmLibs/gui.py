import customtkinter
import cv2
from PIL import Image, ImageTk


class VideoStreamGUI:
    def __init__(self, master):
        self.master = master
        master.title("Video Stream")
        master.geometry("800x600")

        # Configure appearance
        customtkinter.set_appearance_mode("Dark")
        customtkinter.set_default_color_theme("blue")

        # Create video frame
        self.video_frame = customtkinter.CTkFrame(master)
        self.video_frame.pack(pady=20, padx=20, fill="both", expand=True)

        # Video label
        self.video_label = customtkinter.CTkLabel(self.video_frame, text="")
        self.video_label.pack(pady=10, padx=10, fill="both", expand=True)

        # Control buttons
        self.button_frame = customtkinter.CTkFrame(master)
        self.button_frame.pack(pady=10)

        self.start_button = customtkinter.CTkButton(
            self.button_frame, text="Start Stream", command=self.start_stream)
        self.start_button.pack(side="left", padx=5)

        self.stop_button = customtkinter.CTkButton(
            self.button_frame, text="Stop Stream", command=self.stop_stream)
        self.stop_button.pack(side="left", padx=5)

        # Video capture
        self.cap = None
        self.streaming = False

        # For image display
        self.current_image = None

    def start_stream(self):
        if not self.streaming:
            self.cap = cv2.VideoCapture(0)
            self.streaming = True
            self.update_frame()

    def stop_stream(self):
        self.streaming = False
        if self.cap:
            self.cap.release()
            self.cap = None

    def update_frame(self):
        if self.streaming and self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                # Convert to RGB
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                # Resize frame
                frame = cv2.resize(frame, (640, 480))

                # Convert to PIL Image
                img = Image.fromarray(frame)

                # Convert to CTkImage
                ctk_img = customtkinter.CTkImage(
                    dark_image=img,
                    light_image=img,
                    size=(640, 480)
                )

                # Update label
                self.video_label.configure(image=ctk_img)
                self.video_label.image = ctk_img

        # Schedule next update
        if self.streaming:
            self.master.after(10, self.update_frame)

    def on_closing(self):
        self.stop_stream()
        self.master.destroy()


if __name__ == "__main__":
    root = customtkinter.CTk()
    app = VideoStreamGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()