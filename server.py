# # Install minimal dependencies (`torch`, `transformers`, `timm`, `tokenizers`, ...)
# # > pip install -r https://raw.githubusercontent.com/openvla/openvla/main/requirements-min.txt
# from transformers import AutoModelForVision2Seq, AutoProcessor
# from PIL import Image

# import torch

# # Load Processor & VLA
# processor = AutoProcessor.from_pretrained("/home/ubuntu/openvla/runs/checkpoints/openvla-7b+roboracer2_cones_vla+b8+lr-0.0005+lora-r16+dropout-0.0", trust_remote_code=True)
# vla = AutoModelForVision2Seq.from_pretrained(
#     "/home/ubuntu/openvla/runs/checkpoints/openvla-7b+roboracer2_cones_vla+b8+lr-0.0005+lora-r16+dropout-0.0", 
#     attn_implementation="flash_attention_2",  # [Optional] Requires `flash_attn`
#     torch_dtype=torch.bfloat16, 
#     low_cpu_mem_usage=True, 
#     trust_remote_code=True
# ).to("cuda:0")

# # Grab image input & format prompt
# #image: Image.Image = get_from_camera(...)
# image = Image.new("RGB", (224, 224), color=(0, 0, 0))
# prompt = "In: What action should the robot take to {<INSTRUCTION>}?\nOut:"

# # Predict Action (7-DoF; un-normalize for BridgeData V2)
# inputs = processor(prompt, image).to("cuda:0", dtype=torch.bfloat16)
# #action = vla.predict_action(**inputs, unnorm_key="roboracer2_cones_vla", do_sample=False)
# action = vla.predict_action(**inputs, unnorm_key="bridge_orig", do_sample=False)

# #print(action)
# # Execute...


# server.py
from flask import Flask, request, jsonify
import torch, base64
from io import BytesIO
from PIL import Image
from transformers import AutoProcessor, AutoModelForVision2Seq

# Initialize Flask app
app = Flask(__name__)

# Load OpenVLA processor & model
processor = AutoProcessor.from_pretrained(
    "runs/checkpoints/openvla-7b+roboracer2_cones_vla+b8+lr-0.0005+lora-r16+dropout-0.0", 
    trust_remote_code=True
)
vla = AutoModelForVision2Seq.from_pretrained(
    "runs/checkpoints/openvla-7b+roboracer2_cones_vla+b8+lr-0.0005+lora-r16+dropout-0.0",
    attn_implementation="flash_attention_2",
    torch_dtype=torch.bfloat16,
    low_cpu_mem_usage=True,
    trust_remote_code=True
).to("cuda:0").eval()

@app.route('/predict', methods=['POST'])
def predict():
    data = request.get_json()
    instruction = data.get('instruction', '')
    image_b64 = data.get('image', '')
    # Decode base64 image
    img_bytes = base64.b64decode(image_b64)
    image = Image.open(BytesIO(img_bytes)).convert('RGB')

    # Prepare prompt and inputs
    prompt = f"In: What action should the robot take to {instruction}?\nOut:"
    inputs = processor(prompt, image, return_tensors="pt").to("cuda:0", dtype=torch.bfloat16)

    # Run inference
    with torch.no_grad():
        action = vla.predict_action(**inputs, do_sample=False)

    # Convert to Python list
    action_list = action.tolist()
    return jsonify({'action': action_list})

if __name__ == '__main__':
    # Expose on port 5000
    app.run(host='0.0.0.0', port=5000)
