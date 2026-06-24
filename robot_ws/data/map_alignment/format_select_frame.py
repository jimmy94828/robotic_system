from pathlib import Path


INPUT_FILE = Path(__file__).parent / "input" / "selected_frames.txt"
OUTPUT_FILE = Path(__file__).parent / "input" / "selected_frame_formatted.txt"


def format_selected_frames(input_file: Path = INPUT_FILE, output_file: Path = OUTPUT_FILE) -> None:
    formatted_lines = []

    for idx, line in enumerate(input_file.read_text().splitlines()):
        if not line.strip():
            continue

        frame_and_timestamp = line.split()[0]
        original_idx, timestamp = frame_and_timestamp.split("_", 1)
        formatted_lines.append(f"{idx} {original_idx} {timestamp}")

    output_file.write_text("\n".join(formatted_lines) + "\n")


if __name__ == "__main__":
    format_selected_frames()
