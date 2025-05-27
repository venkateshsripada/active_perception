
import config
import base64
import pickle
from PIL import Image
from openai import OpenAI

# Function to encode the image
def encode_image(image_path):
    # print("Encoding image path: ", image_path)
    with open(image_path, "rb") as image_file:
        return base64.b64encode(image_file.read()).decode('utf-8')

def resize_encode_image(image_path):
    # Open the image file
    with Image.open(image_path) as image:
        # Resize the image
        resized_image = image.resize((320, 180), Image.LANCZOS)
        
        # Save the resized image
        resized_path = image_path.replace(".jpg", "_reduced.jpg")  # To avoid overwriting the original file
        resized_image.save(resized_path)
        print("Saved smaller image")

        # Encode the resized image in base64
        with open(resized_path, "rb") as resized_file:
            encoded_string = base64.b64encode(resized_file.read()).decode('utf-8')
        
        return encoded_string
    
def analyze_multiple_images(all_encoded_images, prompt):
    
    message_gpt = [{"role": "system", "content":system_prompt_ap},{"role": "user", "content":[prompt]}]
    
    message_gpt[1]["content"].append({"image":encoded_image1})
    message_gpt[1]["content"].append({"image":encoded_image2})
    # message_gpt[1]["content"].append({"image":encoded_image3})
    # message_gpt[1]["content"].append({"image":in_context_img2_encode})

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
                prompt,
                {"image": encoded_image1},
                {"image": encoded_image2},
            ],
        }
    ],
    temperature = 0.8,
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



def get_summary_section(result_path):

    # Initialize variables to store the result
    summary_lines = []
    output_lines = []

    # Open and read the file
    try:
        with open(result_path, 'r') as file:
            lines = file.readlines()

            inside_summary = False
            inside_output = False

            # Loop through each line to find and store the Summary section
            for line in lines:
                
                # Check for the beginning of the "Summary" section
                if "summary" in line.lower():
                    inside_summary = True
                    continue  # Skip the "Summary" header line itself
                # Check for the start of a new section (e.g., "Thought" or "Output")
                elif "thought" in line.lower():
                    inside_summary = False
                # elif "output" in line.lower():
                #     inside_output = True
                #     continue

                if inside_summary:
                    summary_lines.append(line.strip())  # Strip leading/trailing spaces
                if inside_output:
                    output_lines.append(line.strip())   

            # Join the summary lines into a single output line
            summary = " ".join(summary_lines)
            # output = " ".join(output_lines)

            # arr = output.split("(")
            # coords=  arr[1]
            # coords_x = coords[0:1]
            # coords_y = coords[2:3]
            # coords_z = coords[4:5]

            # position_and_summary = f"I am in currently in ({coords_x},{coords_y},{coords_z})" + summary
            # print(position_and_summary)
            return summary

    except FileNotFoundError:
        with open(result_path, "w") as result_file:
            summary = ""
        print("New result file created")
        return None

def get_vertex_position(result_path):

    summary_lines = []
    with open(result_path, 'r') as file:
        lines = file.readlines()

        inside_summary = False

        # Loop through each line to find and store the Summary section
        for line in lines:
            # Check for the beginning of the "Summary" section
            if "output" in line.lower():
                inside_summary = True

            if inside_summary:
                summary_lines.append(line.strip())  # Strip leading/trailing spaces
        

        # If it is the first view and the VLM did not go to any vertices yet
        if len(summary_lines) == 0:
            print("First loop")
            return ["","",""]
        else:
            # Join the summary lines into a single output line
            summary = " ".join(summary_lines)

            arr = summary.split("(")
            coords=  arr[1]
            coords_x = coords[0:1]
            coords_y = coords[2:3]
            coords_z = coords[4:5]
            coords_array = [coords_x, coords_y, coords_z]

            return coords_array
        

    
def get_prompt():
    with open(local_mem_path + "dict.pkl", "rb") as f:
        local_memory = pickle.load(f)
            
    active_perception_prompt = f"""
    ---------------------------------------------------------------------------------------------
    ---------------------------------------------------------------------------------------------
    Here are two in-context examples:
    ## Example 1:
    Image: [ENCODED_IMAGE_1]  
    Description: Image has stack of wooden blocks on the table
    Input Question: "How can the wooden blocks be stacked better?"
    VLM Thought: To understand how to stack the wooden blocks better, I need a view that shows all the wooden blocks (vertex_z=1). I should look towards the Right of Table(orient_x=1). The wooden blocks are around [4.5,2.3]. As orient_x=1, *move 2 units left* along x = 4.5 - 2 = 2.5
    orient_x: 1
    Output:(2.5,2.3,1) 

    ## Example 2:
    Image: [ENCODED_IMAGE_2]  
    Description: Image has a webcam on a tripod placed on the table
    Input Question: "What brand is the webcam?"
    VLM Thought: To find out the webcam's brand, I need to go close to the table (vertex_z=0). I need to look towards the Left of Table(orient_x=2). The webcam is around [1.0,2.1]. As orient_x=2. *move 2 units right* along x = 1.0 + 2 = 3.0
    orient_x: 2
    Output:(3.0,2.1,0)

    ------------------------------------------------------------------------------------
    -------------------------------------------------------------------------------------
    
    ** That was the end of in-context examples. Now process the following and answer the input question** 

    You are now provided with you past actions below upto the previous time step

    """
    all_time_steps = sorted([k for k in local_memory.keys()])
    current_time_step = all_time_steps[-1]   # Give this to VLM
    
    if len(all_time_steps) != 1:
        for time_step in range(current_time_step):
            output_coords = local_memory[time_step]["output_coords"]
            complete_step_summary = f"\n\n--------In time step {time_step} \n The output suggested was {output_coords}. \
            The orientation along x was {local_memory[time_step]['orientation_x']}. -------\n"
            active_perception_prompt = active_perception_prompt + complete_step_summary
        active_perception_prompt = active_perception_prompt + f"You are currently in time step {current_time_step} -------\n\n"
    else:
        complete_step_summary = "\n\n--------You are currently in time step 0 \n. Hence, there is no action history. --------\n\n"

    active_perception_prompt = active_perception_prompt + f"Text question: {question}"

    return active_perception_prompt



def check_encode_number_images():
    all_images = []
    try:
        encoded_image1 = encode_image(image_path + "grid_image.jpg")
        encoded_image2 = encode_image(image_path + "second_cam_rgb.jpg")
        all_images.append(encoded_image1)
        all_images.append(encoded_image2)
    except FileNotFoundError:
        # print("Only overhead image")
        encoded_image1 = encode_image(image_path + "grid_image.jpg")
        all_images.append(encoded_image1)
    return all_images


def generate_result():
    all_images = check_encode_number_images()

    # with open(target_state_path, "r") as target_pos_file:
    #     target_position = target_pos_file.read()
    # # Add current position to prompt
    # with open(current_position_path, "r") as current_pos_file:
    #     current_position = current_pos_file.read()
    active_perception_prompt = get_prompt()

    result = analyze_multiple_images(all_images, active_perception_prompt)
    # result = analyze_two_images(model, encoded_image1, encoded_image2, active_perception_prompt)
    print(result)

    # Append prompt to log file
    with open(prompt_log_path, "a") as file:
        file.write("\n \n -------Next time step ------------------ \n \n")
        file.write(system_prompt_ap)
        file.write(active_perception_prompt)

    # Append result to log file
    with open(gpt_log_path, "a") as file:
        file.write("\n \n -------Next time step ------------------ \n \n")
        file.write(result)

    # Add result to a file
    with open(gpt_result_path, "w") as file:
        file.write(result)

    
# image_path = "/home/ur_5/ur5_ws/"
# got_both_images = False
model = config.model
scene = config.scene
trial = config.trial
scene_name = config.scene_name

GPT_4okey = config.GPT_4okey
client = OpenAI(api_key=GPT_4okey)

# question = "What is inside the coffee mug?"  # Change the question if required
base_path = config.base_path
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

# prompt_history_path = f"/home/ur_5/Documents/interactive_perception_tests/{scene_name}/scene_{scene}/trial_{trial}/prompt_history_{trial}.txt"

# encoded_image1 = encode_image(image_path + "grid_image.jpg")
# result = analyze_image(model, encoded_image1, initial_prompt)
encoded_image1 = encode_image(base_path + "/In_context_4/grid_image.jpg")
encoded_image2 = encode_image(base_path + "/In_context_5/grid_image.jpg")
# encoded_image3 = encode_image(base_path + "/In_context_3/grid_image.jpg")
# print(result)
system_prompt_ap_shelf = f"""
You are an intelligent embodied agent that can perfrom active perception using visual and textual information. Your job is to analyze the provided image of a tabletop scene based on an input task and text question. The image is captured by a camera mounted on the wrist of Franka Panda robotic manipulator. 

**Your goal is to suggest camera postions and orientations** for the robotic arm to find the target object, with the ultimate aim of gaining more visual information to complete the input task or answer the input question.

The input image will have a 3D cube with 5x5 grids overlaid on it. the grid's x and y vertices will be annotated in the format [x; y]. You must use the below conventions while proving the ouput

Right of Table: Right limit of 3D cube's x-axis i.e (5,0 to 5,0)
Left of Table: Left limit of 3D cube's x-axis i.e (0,0 to 0,5)

Layer Mapping: The 3D cube's z-axis consists of 3 discrete layers that are color coded. They represent the depth.
    - vertex_z = 0 (Blue layer) 35 cm above table surface.
    - vertex_z = 1 (Black layer) 45 cm above the table .
    - vertex_z = 2 (Green layer) is the topmost layer, 55 cm above the table.

- If the Goal Object (or location) is obscured:
    - Identify the object obstructing it the most (the Target Object). 
- If the Goal Object (or location) is visible:
    - Answer the question based on its attributes or position.

Orientation Mapping: This is the fixed orientation of 35 degrees about x-axis (roll in our case).
    - orient_x = 0 . End effector looks perpendicularly down at the table 
    - orient_x = 1 for a +45° roll. End effector looks towards the "Right of Table"
    - orient_x = 2 for a -45° roll. End effector looks towards the "Left of Table"

Action Output consists of the next best camera location: 
    - vertex_x: Positive continuous value along x-axis in the cube coordinate frame rounded to one decimal
    - vertex_y: Positive continuous value along y-axis in the cube coordinate frame rounded to one decimal
    - vertex_z: Integer value along z-axis based on the Layer Mapping
    - orient_x: Integer value based on Orientation Mapping

You will be given two in-context examples which you can use to answer the question

------------------------------------------------

Starting below, you should follow this format for output. Note that there should be no spaces anywhere in "Output" section:

Goal: The goal object or location based on the question
Target object: The target object of interest and what should be done with it to answer the question.
Thought: Using chain-of-thought tell me how to orient the end effector to look at the Goal. Based on that tell me the orient_x value from "Orientation Mapping"
orient_x: Integer value based on "Thought" above
Action Output: Your thinking as to what the next best camera position and orientation are and why 
Output:(vertex_x,vertex_y,vertex_z) 

---------------------------------------------
"""

system_prompt_ap = f"""
You are an intelligent embodied agent that can perfrom active perception using visual and textual information. Your job is to analyze the provided image of a tabletop scene based on an input task and text question. The image is captured by a camera mounted on the wrist of Franka Panda robotic manipulator. 

**Your goal is to suggest camera postions and orientations** for the robotic arm to find the target object, with the ultimate aim of gaining more visual information to complete the input task or answer the input question.

The input image will have a 3D cube with 5x5 grids overlaid on it. the grid's x and y vertices will be annotated in the format [x; y]. You must use the below conventions while proving the ouput

Base of Table: Lower limit of 3D cube's x-axis i.e (0,0 to 5,0)
Top of Table: Upper limit of 3D cube's x-axis i.e (0,5 to 5,5)

Layer Mapping: The 3D cube's z-axis consists of 3 discrete layers that are color coded. They represent the depth.
    - vertex_z = 0 (Blue layer) is at the table surface.
    - vertex_z = 1 (Black layer) is 10 cm above the table .
    - vertex_z = 2 (Green layer) is the topmost layer, 20 cm above the table.

- If the Goal Object (or location) is obscured:
    - Identify the object obstructing it the most (the Target Object). 
- If the Goal Object (or location) is visible:
    - Answer the question based on its attributes or position.

Orientation Mapping: This is the fixed orientation of 35 degrees about x-axis (roll in our case).
    - orient_x = 0 . End effector looks perpendicularly down at the table 
    - orient_x = 1 for a +35° roll. End effector looks towards the "Base of Table"
    - orient_x = 2 for a -35° roll. End effector looks towards the "Top of Table"

Action Output consists of the next best camera location: 
    - vertex_x: Positive continuous value along x-axis in the cube coordinate frame rounded to one decimal
    - vertex_y: Positive continuous value along y-axis in the cube coordinate frame rounded to one decimal
    - vertex_z: Integer value along z-axis based on the Layer Mapping
    - orient_x: Integer value based on Orientation Mapping

You will be given two in-context examples which you can use to answer the question

------------------------------------------------

Starting below, you should follow this format for output. Note that there should be no spaces anywhere in "Output" section:

Goal: The goal object or location based on the question
Target object: The target object of interest and what should be done with it to answer the question.
Thought: Using chain-of-thought tell me how to orient the end effector to look at the Goal. Based on that tell me the orient_x value from "Orientation Mapping"
orient_x: Integer value based on "Thought" above
Action Output: Your thinking as to what the next best camera position and orientation are and why 
Output:(vertex_x,vertex_y,vertex_z) 

---------------------------------------------
"""


# print(full_prompt)
generate_result()