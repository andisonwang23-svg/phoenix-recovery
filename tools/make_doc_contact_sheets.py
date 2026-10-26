from pathlib import Path
from PIL import Image, ImageOps, ImageDraw


def page_number(path):
    return int(path.stem.split("-")[-1])


def build(source, destination, prefix):
    pages = sorted(Path(source).glob("page-*.png"), key=page_number)
    destination = Path(destination)
    destination.mkdir(parents=True, exist_ok=True)
    for start in range(0, len(pages), 4):
        group = pages[start:start + 4]
        thumbs = []
        for page in group:
            image = Image.open(page).convert("RGB")
            image.thumbnail((1200, 1550))
            canvas = Image.new("RGB", (1240, 1620), "white")
            x = (1240 - image.width) // 2
            canvas.paste(image, (x, 40))
            draw = ImageDraw.Draw(canvas)
            draw.text((20, 10), page.stem, fill="black")
            thumbs.append(ImageOps.expand(canvas, border=2, fill="#9aa7b0"))
        sheet = Image.new("RGB", (2488, 3248), "#dfe5e9")
        for index, thumb in enumerate(thumbs):
            sheet.paste(thumb, ((index % 2) * 1244, (index // 2) * 1624))
        end = start + len(group)
        sheet.save(destination / f"{prefix}-{start + 1:02d}-{end:02d}.jpg", quality=92)


build("/private/tmp/phoenix_test_plan_render2", "/private/tmp/doc_contact_sheets2", "test")
build("/private/tmp/pi_drone_guide_render2", "/private/tmp/doc_contact_sheets2", "pi")
