import argparse
from ultralytics import YOLO

parser = argparse.ArgumentParser(description="Convert YOLO model to ONNX format.")
parser.add_argument("--model", type=str, default="yolov11n-seg.pt", help="Path to the YOLO model file.")

def main():
    print("Starting ONNX converter script...")
    args = parser.parse_args()
    model = YOLO(args.model)
    model.export(format="onnx", opset=17)
    print("ONNX conversion completed successfully.")

if __name__ == "__main__":
    main()