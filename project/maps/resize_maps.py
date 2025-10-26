import os, sys
from PIL import Image

folder_src = "original"
folder_destination = "resized"


if __name__ == "__main__":
    for item in os.listdir(folder_src):
        image = Image.open(os.path.join(folder_src, item))
        folder, file = os.path.splitext(folder_src + item)
        imResize = image.resize((30, 30), Image.LANCZOS)
        imResize.save(os.path.join(folder_destination, item), 'PNG', quality=100)
