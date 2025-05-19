import cv2
import numpy as np
import time
from luma.core.interface.serial import spi
from luma.core.render import canvas
from luma.lcd.device import pcd8544
import RPi.GPIO as GPIO

# Setup GPIO
GPIO.setmode(GPIO.BCM)
RST_PIN = 25
LED_PIN = 18  # Define the GPIO pin for the LED

GPIO.setup(LED_PIN, GPIO.OUT)

try:
    # Function to calculate the mean RGB intensity of the ROI in a frame
    def calculate_rgb_intensity(frame, mask):
        roi = cv2.bitwise_and(frame, frame, mask=mask)
        mean_rgb = np.mean(roi[mask > 0], axis=0)
        intensity = np.sum(mean_rgb)
        return intensity, mean_rgb, roi

    # Function to convert intensity to percentage
    def intensity_to_percentage(intensity):
        max_possible_intensity = 3 * 255
        percentage = (intensity / max_possible_intensity) * 100
        return percentage

    # Function to find the mask of the blue regions
    def find_blue_mask(frame):
        # Convert the frame to the HSV color space
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Define the lower and upper bounds for the blue color
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])
        # Create a mask for the blue color
        mask = cv2.inRange(hsv_frame, lower_blue, upper_blue)
        return mask

    # Open the camera
    cap = cv2.VideoCapture(0)  # 0 is usually the ID for the default camera

    # Check if the camera opened successfully
    if not cap.isOpened():
        print("Error opening camera")
        exit()
    
    # Turn on the LED when the camera opens
    GPIO.output(LED_PIN, GPIO.HIGH)

    # Initialize variables
    max_intensity = 0
    max_mean_rgb = np.zeros(3)
    intensity_list = []
    mean_rgb_list = []
    frame_with_max_intensity = None
    roi_with_max_intensity = None

    # Start the timer
    start_time = time.time()

    # Process each frame in the video
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Check if 20 seconds have passed
        elapsed_time = time.time() - start_time
        if elapsed_time > 20:
            break

        # Find the mask of blue pixels
        mask = find_blue_mask(frame)

        # Calculate the intensity within the ROI
        frame_intensity, mean_rgb, roi_frame = calculate_rgb_intensity(frame, mask)
        intensity_list.append(frame_intensity)
        mean_rgb_list.append(mean_rgb)

        if frame_intensity > max_intensity:
            max_intensity = frame_intensity
            max_mean_rgb = mean_rgb
            frame_with_max_intensity = frame
            roi_with_max_intensity = roi_frame

        # Find contours of the blue regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Draw contours around the ROI
            frame_with_contours = frame.copy()
            cv2.drawContours(frame_with_contours, contours, -1, (0, 255, 0), 2)
            
            # Create a window of the same size as the frame with contours
            window_name = 'Frame with ROI'
            #cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window_name, frame.shape[1] // 2, frame.shape[0] // 2)
            
            # Display the frame with contours and intensity
            intensity_text = f"Intensity: {frame_intensity:.2f}"
            cv2.putText(frame_with_contours, intensity_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1, cv2.LINE_AA)
            #cv2.imshow(window_name, frame_with_contours)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    
    # Turn off the LED when the camera closes
    GPIO.output(LED_PIN, GPIO.LOW)

    max_intensity_percentage = intensity_to_percentage(max_intensity)
    print(f"Maximum intensity in the ROI due to chemiluminescence: {max_intensity:.2f}")
    print(f"Mean RGB values for maximum intensity frame: {max_mean_rgb}")
    print(f"Maximum intensity in percentage: {max_intensity_percentage:.2f}%")

    # Find the pixel with the highest intensity in the ROI with max intensity
    max_pixel_intensity = 0
    max_pixel_coords = (0, 0)

    for i in range(roi_with_max_intensity.shape[0]):
        for j in range(roi_with_max_intensity.shape[1]):
            pixel_intensity = np.sum(roi_with_max_intensity[i, j])
            if pixel_intensity > max_pixel_intensity:
                max_pixel_intensity = pixel_intensity
                max_pixel_coords = (i, j)

    # Calculate the average intensity of the surrounding pixels and store their coordinates
    surrounding_pixel_intensities = []
    surrounding_pixel_coords = []

    for i in range(max_pixel_coords[0]-1, max_pixel_coords[0]+2):
        for j in range(max_pixel_coords[1]-1, max_pixel_coords[1]+2):
            if i >= 0 and i < roi_with_max_intensity.shape[0] and j >= 0 and j < roi_with_max_intensity.shape[1]:
                surrounding_pixel_intensities.append(np.sum(roi_with_max_intensity[i, j]))
                surrounding_pixel_coords.append((i, j))

    average_surrounding_intensity = np.mean(surrounding_pixel_intensities)
    average_surrounding_intensity_percentage = intensity_to_percentage(average_surrounding_intensity)

    # Display the results
    print(f"Coordinates of the pixel with the highest intensity: {max_pixel_coords}")
    print(f"Maximum pixel intensity in the ROI: {max_pixel_intensity:.2f}")
    print(f"Average surrounding pixel intensity in the ROI: {average_surrounding_intensity:.2f}")
    print(f"Average surrounding pixel intensity in percentage: {average_surrounding_intensity_percentage:.2f}%")
    print(f"Coordinates of the surrounding pixels: {surrounding_pixel_coords}")

    # Initialize the SPI interface and the display
    serial = spi(port=0, device=0, gpio=GPIO, rst=RST_PIN)
    device = pcd8544(serial, rotate=0)

    # Display the maximum intensity percentage on the LCD
    with canvas(device) as draw:
        draw.text((0, 0), f"Max Intensity:", fill=1)
        draw.text((0, 10), f"{max_intensity:.2f}", fill=1)
        draw.text((0, 20), f"Max Intensity %:", fill=1)
        draw.text((0, 30), f"{max_intensity_percentage:.2f}%", fill=1)
        
finally:
    if GPIO.getmode() is not None:
        GPIO.cleanup()
