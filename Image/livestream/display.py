import os
import re
import argparse


def get_images(path):
    images = {"images": []}
    for file in os.listdir(path):
        if file.endswith((".jpg", ".png", ".jpeg")):
            images["images"].append(file)
    return images


def sort_numbered_filenames(filenames):
    """Sort locked_1.jpg / frame_000001.jpg etc by the last number in the name."""
    def key(f):
        m = re.findall(r"(\d+)", f)
        return int(m[-1]) if m else 0
    return sorted(filenames, key=key)


def create_html_content(images, base_href, title="Live Detect Viewer", refresh_sec=0):
    html = """
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>{title}</title>
    {refresh_tag}
    <style>
        body { font-family: sans-serif; margin: 20px; background-color: #f4f4f4; }
        h1 { text-align: center; color: #333; }
        h2 { color: #555; border-bottom: 2px solid #ddd; padding-bottom: 10px; }
        .container {
            display: flex;
            flex-wrap: wrap;
            justify-content: flex-start;
            gap: 20px;
        }
        .image-box {
            border: 1px solid #ccc;
            border-radius: 8px;
            padding: 15px;
            background-color: #fff;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            text-align: center;
            width: 400px;
        }
        .image-box h3 { margin-top: 0; font-size: 1em; color: #666; }
        img {
            max-width: 100%;
            height: auto;
            border-radius: 4px;
        }
    </style>
</head>
<body>
    <h1>{title}</h1>
""".format(
        title=title,
        refresh_tag=(f'<meta http-equiv="refresh" content="{int(refresh_sec)}">' if refresh_sec and int(refresh_sec) > 0 else ""),
    )

    image_files = sort_numbered_filenames(images["images"]) if images["images"] else []
    if image_files:
        html += '<h2>Images</h2>\n<div class="container">\n'
        for image_file in image_files:
            src = f"{base_href.rstrip('/')}/{image_file}"
            html += f"""
            <div class="image-box">
                <h3>{image_file}</h3>
                <img src="{src}" alt="{image_file}">
            </div>"""
        html += "\n</div>\n"
    else:
        html += '<h2>Images</h2>\n<div class="container"><p style="color:#888;">No images yet. Run a lock to save locked_1.jpg / locked_2.jpg.</p></div>\n'

    html += """
</body>
</html>
"""
    return html


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate a simple HTML viewer for a folder of images.")
    parser.add_argument("--input-dir", default="../live_detect", help="Folder containing images (default: ../live_detect)")
    parser.add_argument("--output", default="pokemon.html", help="Output HTML file (default: pokemon.html)")
    parser.add_argument("--title", default="Live Detect Viewer", help="Page title")
    parser.add_argument("--refresh", type=int, default=2, help="Auto-refresh seconds (0 = off). Default: 2")
    args = parser.parse_args()

    images_path = args.input_dir
    images = get_images(images_path)
    html_content = create_html_content(images, base_href=images_path, title=args.title, refresh_sec=args.refresh)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(html_content)
    print(f"{args.output} generated successfully from {images_path}.")
