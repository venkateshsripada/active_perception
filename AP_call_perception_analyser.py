
import pdb
import config
import base64
import pickle
from openai import OpenAI


# Function to encode the image
def encode_image(image_path):
#   print("Encoding image path: ", image_path)
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')

def analyze_multiple_images(all_encoded_images, prompt):
    message_gpt = [{"role": "system", "content":system_prompt},{"role": "user", "content":[prompt]}]
    
    for encoded_image_ in all_encoded_images:
        message_gpt[1]["content"].append({"image":encoded_image_})
       
    response = client.chat.completions.create(
    model=model,
    messages=message_gpt,
    temperature = 0.0,
    max_tokens = 1000,
    )
    
    return response.choices[0].message.content

def analyze_two_images(model, encoded_image1, encoded_image2, prompt):
    response = client.chat.completions.create(
    model=model,
    messages=[
        {
            "role": "user",
            "content": [
                    {
                        "type": "text",
                        "text": prompt
                    },
                    {
                        "type":"image_url",
                        "image_url": {
                            "url":  f"data:image/jpeg;base64,{encoded_image1}",
                            "detail": "auto"
                            },

                        "type":"image_url",
                        "image_url": {
                            "url":  f"data:image/jpeg;base64,{encoded_image2}",
                            "detail": "auto"
                        },
                    },
            ],
        }
    ],
    temperature = 0.0,
    max_tokens = 1000,
    )

    return response.choices[0].message.content

def analyze_image(model, encoded_image1, prompt):
    response = client.chat.completions.create(
    model=model,
    messages=[
        {
            "role": "user",
            "content": [
                    {
                        "type": "text",
                        "text": prompt
                    },
                    {
                        "type":"image_url",
                        "image_url": {
                            "url":  f"data:image/jpeg;base64,{encoded_image1}",
                            "detail": "auto"
                            },
                    },
            ],
        }
    ],
    temperature = 0.0,
    max_tokens = 1000,
    )

    return response.choices[0].message.content

def check_encode_number_images():
    all_images = []

    try:
        encoded_image1 = encode_image(image_path + "saved_image.jpg")
        encoded_image2 = encode_image(image_path + "second_cam_rgb.jpg")
        all_images.append(encoded_image1)
        all_images.append(encoded_image2)
    except FileNotFoundError:
        # print("Only overhead image")
        encoded_image1 = encode_image(image_path + "saved_image.jpg")
        all_images.append(encoded_image1)

    return all_images

def create_empty_files():
    with open(target_state_path, "w") as target_pos_file:
        pass
    
    with open(current_position_path, "w") as current_pos_file:
        pass


def generate_result():
    user_prompt = f" Text question: {question}"

    with open(local_mem_path + "dict.pkl", "rb") as f:
        local_memory = pickle.load(f)
            
    all_time_steps = sorted([k for k in local_memory.keys()])
    current_time_step = all_time_steps[-1]   # Give this to VLM
    if current_time_step == 5:
        time_up_prompt = "\n\n**You have performed maximum number of iterations**. **Provide a conclusive answer now**"
        user_prompt = user_prompt + time_up_prompt

    all_images = check_encode_number_images()

    # Give input to VLM
    if len(all_images) == 1:
        create_empty_files()

    result = analyze_multiple_images(all_images, user_prompt)
    print(result)

    # Append prompt to log file
    with open(prompt_log_path, "a") as file:
        file.write("\n \n -------Perception Analyser prompt ------------------ \n \n")
        file.write(system_prompt)
        file.write(user_prompt)

    # Append result to log file
    with open(gpt_log_path, "a") as file:
        file.write("\n \n ------Perception Analyser prompt ------------------ \n \n")
        file.write(result)

    # Add result to a file
    with open(gpt_analyser_path, "w") as file:
        file.write(result)
    

model = config.model
scene = config.scene
trial = config.trial
scene_name = config.scene_name

GPT_4okey = config.GPT_4okey
client = OpenAI(api_key=GPT_4okey)

image_path = config.image_path
image_save_path=  config.image_save_path
local_mem_path = config.memory_dictionary_path

question = config.question

current_position_path = config.current_position_path
target_state_path = config.target_state_path

prompt_log_path = config.prompt_log_path
gpt_result_path = config.result_path
gpt_analyser_path = config.gpt_analyser_path

gpt_log_path = config.gpt_log_path

system_prompt = f"""

You are an intelligent embodied agent that can thoroughly perform object detection. Your job is to analyze the provided image and precisely answer the input text question. The image is captured by a camera mounted on a Franka Panda robotic manipulator.

Upon analysing the input image and text question you are to consider:
- Given your current state, are you able to provide a conclusive answer to the input question?
If yes, you should stop. output the "Answer:" and "output:stop" in a new line  
If no, you should "output:go" and continue:

**Important: Make sure to thoroughly inspect under and around all objects**

**Do not guess**. If you are unsure about answering the text question, "output:go" and a new image will be captured from a better location.

------------------------------------------------

Starting below, you should follow this format for output:

Objects: All the objects on the table
Target object: The target object of interest to answer the question
Thought: Use chain-of-thought and iteratively reason all objects on the table. Always think about what answer you give and why
Conclusive Answer: Are you able to give a thorough answer to the question (Yes/No)
... If you conclusively answered Yes 
Answer: Tell what the answer to the input question is
Output:stop
... If you conclusively answered No
Output:go

---------------------------------------------

"""
generate_result()