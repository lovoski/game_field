# Import necessary libraries
import os
import sys
import argparse
from PIL import Image  # Pillow library for image handling
from fpdf import FPDF   # fpdf2 library for PDF creation

def create_pdf_from_images(image_dir_path):
    """
    Finds images in a directory and converts them into a single PDF file.

    Args:
        image_dir_path (str): The path to the directory containing images.

    Returns:
        bool: True if PDF creation was attempted (even if some images failed),
              False if the input directory was invalid or no images were found.
    """
    # --- Configuration ---
    allowed_extensions = ('.png', '.jpg', '.jpeg', '.webp')
    # --- End Configuration ---

    # Validate and normalize the input directory path
    if not os.path.isdir(image_dir_path):
        print(f"Error: The specified path '{image_dir_path}' is not a valid directory.")
        return False

    abs_image_dir = os.path.abspath(image_dir_path)
    print(f"Processing images from directory: {abs_image_dir}")

    # Determine output PDF name and path
    # Use the directory name for the PDF file name
    dir_name = os.path.basename(os.path.normpath(abs_image_dir))
    output_pdf_name = f"{dir_name}.pdf"
    # Save the PDF in the parent directory of the image folder
    output_dir = os.path.dirname(abs_image_dir)
    # If the input is the root directory, save in the current working directory instead
    if output_dir == abs_image_dir:
        print("Warning: Input directory appears to be the root. Saving PDF in the current working directory.")
        output_dir = '.' # Current working directory
    output_pdf_path = os.path.join(output_dir, output_pdf_name)


    # List to hold the paths of the image files found
    image_files = []

    # Scan the specified directory for image files
    print(f"Scanning for images...")
    try:
        for filename in sorted(os.listdir(abs_image_dir)): # Sort filenames alphabetically
            # Check if the file has one of the allowed extensions (case-insensitive)
            if filename.lower().endswith(allowed_extensions):
                full_path = os.path.join(abs_image_dir, filename)
                # Ensure it's a file, not a directory
                if os.path.isfile(full_path):
                    image_files.append(full_path)
                    # print(f"  Found image: {filename}") # Optional: uncomment for verbose listing
    except Exception as e:
        print(f"An error occurred while scanning the directory: {e}")
        return False


    if not image_files:
        print("No image files with supported extensions (.png, .jpg, .jpeg, .webp) found.")
        return False

    print(f"\nFound {len(image_files)} image(s). Preparing to create PDF: '{output_pdf_path}'")

    # Create instance of FPDF class (A4 Portrait by default)
    pdf = FPDF(orientation='P', unit='mm', format='A4')
    pdf.set_auto_page_break(auto=False)

    # --- Constants for PDF sizing (A4 in mm) ---
    A4_WIDTH_MM = 210
    A4_HEIGHT_MM = 297
    MARGIN_MM = 10 # 10mm margin on each side
    MAX_IMG_WIDTH_MM = A4_WIDTH_MM - 2 * MARGIN_MM
    MAX_IMG_HEIGHT_MM = A4_HEIGHT_MM - 2 * MARGIN_MM
    # --- End Constants ---

    processed_count = 0
    error_count = 0
    # Iterate over the found image files
    for image_path in image_files:
        print(f"Processing '{os.path.basename(image_path)}'...")
        try:
            # Open the image file using Pillow
            with Image.open(image_path) as img:
                # Convert image to RGB if necessary (avoids issues with transparency/palettes)
                if img.mode != 'RGB':
                    try:
                        img = img.convert('RGB')
                    except Exception as convert_err:
                        print(f"  Warning: Could not convert '{os.path.basename(image_path)}' to RGB ({convert_err}). Skipping file.")
                        error_count += 1
                        continue

                # Get image dimensions
                width_px, height_px = img.size
                if width_px <= 0 or height_px <= 0:
                    print(f"  Warning: Image '{os.path.basename(image_path)}' has invalid dimensions (<=0). Skipping file.")
                    error_count += 1
                    continue

                # Calculate aspect ratio
                aspect_ratio = width_px / height_px

                # Determine the image size in the PDF to fit within margins while preserving aspect ratio
                if (MAX_IMG_WIDTH_MM / aspect_ratio) <= MAX_IMG_HEIGHT_MM:
                    # Width is the limiting factor
                    img_width_mm = MAX_IMG_WIDTH_MM
                    img_height_mm = img_width_mm / aspect_ratio
                else:
                    # Height is the limiting factor
                    img_height_mm = MAX_IMG_HEIGHT_MM
                    img_width_mm = img_height_mm * aspect_ratio

                # Center the image on the page
                x_pos = (A4_WIDTH_MM - img_width_mm) / 2
                y_pos = (A4_HEIGHT_MM - img_height_mm) / 2

                # Add a new page for each image
                pdf.add_page()
                # Add the image to the PDF page
                pdf.image(image_path, x=x_pos, y=y_pos, w=img_width_mm, h=img_height_mm)
                processed_count += 1
                print(f"  Added '{os.path.basename(image_path)}' to PDF.")

        except FileNotFoundError:
            print(f"  Error: Image file not found during processing '{image_path}'. Skipping.")
            error_count += 1
        except Image.UnidentifiedImageError:
            print(f"  Error: Cannot identify image file '{os.path.basename(image_path)}'. It might be corrupted or not a valid image format supported by Pillow. Skipping.")
            error_count += 1
        except Exception as e:
            # Catch potential FPDF errors or other unexpected issues
            print(f"  An unexpected error occurred while processing '{os.path.basename(image_path)}': {e}")
            error_count += 1

    # Save the PDF file
    if processed_count > 0:
        try:
            pdf.output(output_pdf_path, "F")
            print(f"\nSuccessfully created PDF: '{output_pdf_path}' with {processed_count} image(s).")
            if error_count > 0:
                print(f"Note: {error_count} image(s) could not be processed due to errors.")
            return True
        except Exception as e:
            print(f"\nError: Failed to save the PDF file '{output_pdf_path}': {e}")
            return False
    else:
        print("\nNo images were successfully processed to create a PDF.")
        return False

# --- Main Execution ---
if __name__ == "__main__":
    # Set up argument parser
    parser = argparse.ArgumentParser(
        description="Convert all images (.png, .jpg, .webp) in a specified directory into a single PDF file. "
                    "The PDF is named after the directory and saved in its parent directory."
    )
    # Add the required positional argument for the image directory
    parser.add_argument(
        "image_directory",
        help="Path to the directory containing the image files."
    )

    # Parse the command-line arguments
    args = parser.parse_args()

    # Call the main function with the provided directory path
    success = create_pdf_from_images(args.image_directory)

    print("\nScript finished.")
    # Exit with appropriate status code
    sys.exit(0 if success else 1)