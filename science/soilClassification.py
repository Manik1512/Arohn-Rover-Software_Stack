import tkinter as tk
from tkinter import filedialog
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import numpy as np
import time
from tensorflow.keras.models import load_model

# Load ML model
model_fp = r"/home/manik/Downloads/soil3.h5"
model = load_model(model_fp)

# Soil classes
classes = ["Alluvial soil", "Black soil", "Desert soil", "Laterite soil", "Red soil"]

def preprocess_image(cropped_img):
    """Preprocess an image before passing it to the model."""
    cropped_img = cv2.resize(cropped_img, (256, 256))  # Resize to model input size (256, 256)
    image_array = cropped_img / 255.0  # Normalize
    img_batch = np.expand_dims(image_array, axis=0)  # Add batch dimension
    return img_batch

def model_classify(cropped_img):
    """Classify a single cropped image using the model."""
    img_batch = preprocess_image(cropped_img)
    prediction_array = model.predict(img_batch)[0]
    predicted_idx = np.argmax(prediction_array)
    return classes[predicted_idx], prediction_array

def classify_images(image_fp):
    """Classify an image by segmenting it into smaller parts."""
    img = cv2.imread(image_fp)
    img = cv2.resize(img, (1024, 1024))  # Resize to 1024x1024
    im_dim = 256  # Crop size
    
    counts = {soil: 0 for soil in classes}
    overall_predictions = []
    
    for r in range(0, img.shape[0], im_dim):
        for c in range(0, img.shape[1], im_dim):
            cropped_img = img[r:r + im_dim, c:c + im_dim, :]
            h, w, _ = cropped_img.shape
            if h == im_dim and w == im_dim:
                classification, prediction_array = model_classify(cropped_img)
                counts[classification] += 1
                overall_predictions.append(prediction_array)
    
    total = sum(counts.values())
    proportions = {soil: round((count / total) * 100, 2) for soil, count in counts.items() if total > 0}
    
    # Get the highest probability class
    if overall_predictions:
        avg_predictions = np.mean(overall_predictions, axis=0)
        highest_idx = np.argmax(avg_predictions)
        highest_class = classes[highest_idx]
        highest_prob = round(avg_predictions[highest_idx] * 100, 2)
    else:
        highest_class, highest_prob = "Unknown", 0.0
    
    return proportions, highest_class, highest_prob

def classify_percentage(image_fp):
    """Get classification results as a formatted string."""
    start = time.time()
    proportions, highest_class, highest_prob = classify_images(image_fp)
    finish = round(time.time() - start, 5)
    
    results = "\n".join([f"Percent {soil}: {percent}%" for soil, percent in proportions.items()])
    return f"---\n{results}\n\nSoil Type with the Highest Confidence: {highest_class} ({highest_prob}%)\nTime to Classify: {finish} seconds\n---"

def load_image():
    global image_fp
    image_fp = filedialog.askopenfilename(filetypes=[("Image files", ".jpg;.png;*.jpeg")])
    if image_fp:
        img = Image.open(image_fp)
        img = img.resize((256,256), Image.Resampling.LANCZOS)
        img_tk = ImageTk.PhotoImage(img)
        image_label.configure(image=img_tk, text="", width=350, height=350)
        image_label.image = img_tk

def update_results():
    if image_fp:
        result_text.set(classify_percentage(image_fp))

# GUI Setup
root = tk.Tk()
root.title("ML Model Results GUI")
root.geometry("1000x800")
root.configure(bg="#1e1e1e")

style = ttk.Style()
style.configure("TFrame", background="#1e1e1e")
style.configure("TButton", font=("Helvetica", 12), padding=10, background="#4CAF50", foreground="white")
style.map("TButton", background=[("active", "#66bb6a")])

image_frame = ttk.Frame(root)
image_frame.pack(side=tk.TOP, expand=False, pady=20)

image_label = tk.Label(image_frame, text="Image Preview", bg="#2c2c2c", fg="white", width=40, height=20, font=("Helvetica", 14), relief="solid")
image_label.pack(pady=10)

results_frame = ttk.Frame(root, padding="10")
results_frame.pack(side=tk.BOTTOM, fill=tk.X, expand=False)

result_text = tk.StringVar()
results_label = tk.Label(results_frame, textvariable=result_text, font=("Helvetica", 14), justify=tk.LEFT, bg="#2c2c2c", fg="white", anchor="nw", relief="sunken", width=70, height=10, bd=2, padx=10, pady=10)
results_label.pack(padx=10, pady=10, fill=tk.BOTH, expand=False)

button_frame = ttk.Frame(root, padding="10")
button_frame.pack(side=tk.BOTTOM, fill=tk.X)

load_button = ttk.Button(button_frame, text="Load Image", command=load_image)
load_button.pack(side=tk.LEFT, padx=10)

update_button = ttk.Button(button_frame, text="Classify Image", command=update_results)
update_button.pack(side=tk.LEFT, padx=10)

root.mainloop()
