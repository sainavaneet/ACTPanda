import cv2
import h5py
import numpy as np
import tkinter as tk
import logging
import threading

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

class EpisodePlayer(threading.Thread):
    def __init__(self, hdf5_path, fps):
        super().__init__()
        self.hdf5_path = hdf5_path
        self.fps = fps
        self.stop_event = threading.Event()

    def run(self):
        try:
            with h5py.File(self.hdf5_path, 'r') as file:
                images = file['observations/images/top/']
                interval = int(1000 / self.fps)
                for i in range(images.shape[0]):
                    if self.stop_event.is_set():
                        break
                    image = images[i]
                    if not isinstance(image, np.ndarray) or image.ndim != 3:
                        image_height, image_width = 480, 640
                        image = image.reshape((image_height, image_width, 3))
                    cv2.imshow('Episode Image Replay', image)
                    if cv2.waitKey(interval) & 0xFF == ord('q'):
                        break
                cv2.destroyAllWindows()
        except Exception as e:
            logging.error("Failed to replay episode images: %s", e)
            cv2.destroyAllWindows()

    def stop(self):
        self.stop_event.set()
        cv2.destroyAllWindows()

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Episode Player")
        self.episode_no = 0
        self.base_path = 'dataset/episode_{}.hdf5'
        self.player = None

        self.episode_label = tk.Label(self.root, text=f"Episode: {self.episode_no}")
        self.episode_label.pack(pady=20)

        self.dataset_info_text = tk.Text(self.root, height=10, width=50)
        self.dataset_info_text.pack(pady=20)

        self.play_button = tk.Button(self.root, text="Play Episode", command=self.play_episode)
        self.play_button.pack(pady=10)

        self.stop_button = tk.Button(self.root, text="Stop Replay", command=self.stop_replay)
        self.stop_button.pack(pady=10)

        self.prev_button = tk.Button(self.root, text="Previous Episode", command=self.prev_episode)
        self.prev_button.pack(pady=10, side=tk.LEFT)

        self.next_button = tk.Button(self.root, text="Next Episode", command=self.next_episode)
        self.next_button.pack(pady=10, side=tk.RIGHT)

        self.update_episode_label()
        self.show_dataset_info()

    def update_episode_label(self):
        self.episode_label.config(text=f"Episode: {self.episode_no}")
        self.root.update_idletasks()  # Ensure the label is updated immediately
        self.show_dataset_info()  # Update dataset info whenever episode changes

    def get_episode_path(self):
        return self.base_path.format(self.episode_no)

    def play_episode(self):
        self.stop_replay()  # Stop any existing replay before starting a new one
        hdf5_path = self.get_episode_path()
        logging.info(f"Playing episode from {hdf5_path}")
        try:
            with h5py.File(hdf5_path, 'r'):
                self.player = EpisodePlayer(hdf5_path, fps=30)
                self.player.start()
        except Exception as e:
            logging.error(f"Could not open file {hdf5_path}: {e}")

    def next_episode(self, event=None):
        self.episode_no += 1
        self.update_episode_label()
        self.play_episode()

    def prev_episode(self, event=None):
        if self.episode_no > 0:
            self.episode_no -= 1
            self.update_episode_label()
            self.play_episode()

    def stop_replay(self):
        if self.player and self.player.is_alive():
            self.player.stop()
            self.player.join()
        cv2.destroyAllWindows()

    def show_dataset_info(self):
        hdf5_path = self.get_episode_path()
        try:
            with h5py.File(hdf5_path, 'r') as file:
                dataset_info = []
                def retrieve_info(name, obj):
                    if isinstance(obj, h5py.Dataset):
                        dataset_info.append(f"{name}: shape={obj.shape}, dtype={obj.dtype}")

                file.visititems(retrieve_info)
                info_message = "\n".join(dataset_info)
                self.dataset_info_text.delete(1.0, tk.END)
                self.dataset_info_text.insert(tk.END, info_message)
        except Exception as e:
            logging.error(f"Could not open file {hdf5_path}: {e}")
            self.dataset_info_text.delete(1.0, tk.END)
            self.dataset_info_text.insert(tk.END, f"Could not open file {hdf5_path}: {e}")

if __name__ == '__main__':
    root = tk.Tk()
    app = App(root)
    root.mainloop()
