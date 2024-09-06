import openai
import base64
from openai import OpenAI

# GPT_4okey = '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'
GPT_4okey = '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'
client = OpenAI(api_key=GPT_4okey)


# Function to encode the image
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')

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
        

    
def get_prompt(current_position, target_position):
    active_perception_prompt = f"""
        You are an intelligent embodied agent that can perfrom active perception using visual and textual information. Your job is to analyze the provided image of a tabletop scene given an input text question. The image is captured by a camera mounted on a UR5 robotic manipulator. 

        The input image will have a 3D cube with 3x3 grids overlaid on it. the grid's x and y vertices will be annotated in the format [x; y]. You must use the below conventions while proving the ouput

        Base of tbale: Lower limit of 3D cube's x-axis i.e (0,0 to 3,0)
        Top of table: Upper limit of 3D cube's x-axis i.e (0,3 to 3,3)

        Layer Mapping: The 3D cube's z-axis consists of 3 discrete layers that are color coded. They represent the depth.
            - vertex_z = 0 (Blue layer) is at the table surface.
            - vertex_z = 1 (Black layer) is 10 cm above the table .
            - vertex_z = 2 (Green layer) is the topmost layer, 20 cm above the table.
            
        Orientation Mapping: This is the fixed orientation of 35 degrees about x-axis (roll in our case).
            - orient_x = 0 when the target object of interest is flat on the table and perpendicular to the table 
            - orient_x = 1 for a +35° roll. Target is inclined towards the Base of Table
            - orient_x = 2 for a -35° roll. Target is inclined towards the Top of Table

        You take actions by suggesting new camera positions and orientations with respect to 3D cube coordinate frame. The camera has an offset only along the z-axis and stops 35 cm above a given vertex_z. You can suggest new meaningful camera positions and orientations to gain more visual information with the ultimate goal of being able to answer the input text question.

        At each step you will be given your "Current State" i.e current position and orientation in the 3D cube's coordinate frame. You will be given an inital estimate of the target object location called "Target State"

        Upon analysing the input image, states and text question you are to consider:
        1) Given your current state, are you able to provide a conclusive answer to the input question?
        If yes, you should stop. output the "Answer:" and "output:stop" in a new line  
        If no, you should continue:
        2) Consider where you and the target object are and whether changing just your Orientation gives a better view to answer the question. Suggest a new position and orientation and show your reasoning in the Thought section 
        3) Provide "Action Output"

        Action Output consists of the next best camera location: 
            - vertex_x: Positive continuous value along x-axis in the cube coordinate frame rounded to one decimal
            - vertex_y: Positive continuous value along y-axis in the cube coordinate frame rounded to one decimal
            - vertex_z: Integer value along z-axis based on the Layer Mapping
            - orient_x: Integer value based on Orientation Mapping

        ------------------------------------------------

        Staring below, you should follow this format for output. Note that there should be no spaces anywhere in "Output" section:

        Target object: The target object of interest and what should be done with it to answer the question.
        Thought: Always think about what to do next and why
        Conclusive Answer: Are you able to give a thorough answer to the question (Yes/No)
        ... If you conclusively answered Yes 
        Answer: Tell what the answer to the input question is
        Output:stop
        ... If you conclusively answered No
        Action Output: Your thinking as to what the next best camera position and orientation are and why 
        Output:(vertex_x,vertex_y,vertex_z,orient_x) 

        ---------------------------------------------

        Current State: {current_position}
        Target State: {target_position}
        Text question: {question}
        """

    return active_perception_prompt


def check_encode_number_images():
    try:
        encoded_image1 = encode_image(image_path + "grid_image_overhead.jpg")
        encoded_image2 = encode_image(image_path + "grid_image.jpg")
        return encoded_image1, encoded_image2
    except FileNotFoundError:
        print("Only overhead image")
        encoded_image1 = encode_image(image_path + "grid_image_overhead.jpg")
        return encoded_image1, None


def generate_result():
    encoded_image1, encoded_image2 = check_encode_number_images()
    
    # Get only the Summary from gpt_log
    # summary = get_summary_section(gpt_result_path)
    # coord_array = get_vertex_position(gpt_result_path)
    # # if not coord_x:

    
    
    # # Add summary to prompt history context
    # if summary != None:
    #     with open(prompt_history_path, "a") as file:
    #         file.write("\n")
    #         file.write(summary)
    # else:
    #     with open(prompt_history_path, "w") as file:
    #         print("New prompt history log file")

    # # # Get iterated prompt + history
    # # try:
    # with open(prompt_history_path, "r") as history_file:
    #     history_data = history_file.read()  
    #     full_prompt = get_prompt(history_data, coord_array)
    # except FileNotFoundError:
    #     with open(prompt_history_path, "w") as history_file:          
    #         print("Created new file")
    

    # Give input to VLM
    if encoded_image2 != None:
        # Add target position to prompt
        with open(target_state_path, "r") as target_pos_file:
            target_position = target_pos_file.read()

        # Add current position to prompt
        with open(current_position_path, "r") as current_pos_file:
            current_position = current_pos_file.read()

        # Append prompt to log file
        with open(prompt_log_path, "a") as file:
            file.write("\n \n -------Next time step ------------------ \n \n")
            file.write(active_perception_prompt)

        active_perception_prompt = get_prompt(current_position, target_position)
        result = analyze_two_images(model, encoded_image1, encoded_image2, active_perception_prompt)

        # Append result to log file
        with open(gpt_log_path, "a") as file:
            file.write("\n \n -------Nextt time step ------------------ \n \n")
            file.write(result)

    
    else:
        # Append prompt to log file
        with open(prompt_log_path, "a") as file:
            file.write("\n \n -------First time step (Analyser prompt) ------------------ \n \n")
            file.write(initial_prompt)

        result = analyze_image(model, encoded_image1, initial_prompt)
        print(result)

        # Append result to log file
        with open(gpt_log_path, "a") as file:
            file.write("\n \n -------First time step (Analyser prompt) ------------------ \n \n")
            file.write(result)

    # Add result to a file
    with open(gpt_result_path, "w") as file:
        file.write(result)

    

    


model = "gpt-4o"
# image_path = "/home/ur_5/ur5_ws/"
# got_both_images = False

trial = 11
prompt_history_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/prompt_history_{trial}.txt"
prompt_log_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/prompt_log_{trial}.txt"
gpt_result_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/gpt_result_{trial}.txt"
gpt_log_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/gpt_log_{trial}.txt"

current_position_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/current_position_{trial}.txt"
target_state_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/target_position_{trial}.txt"

image_path = f"/home/ur_5/Documents/gpt_tests/Test_5_9_24/dual_prompt/test_{trial}/"


question = "What is inside the red coffe mug?"

new_prompt = f"""
Analyze the provided image of a tabletop scene captured by a camera mounted on a UR5 robotic manipulator. The scene includes various objects and an augmented 3D cube with grid vertices 15 cm apart, where the origin (0,0,0) aligns with the Aruco marker ID 8 close to the base of the robot. The first image is encoded as 'encoded_image1' and provides an overhead view from 80 cm above the surface of the table. Note any objects that appear partially or fully occluded. Use the grid structure to specify the coordinates and extent of any occlusion.

The cube's z-axis is divided into color-coded layers: 0 represents the blue layer (bottom-most on table's surface), 1 represents the black layer, and 2 represents the green layer (top-most). The camera can also provide views from a different orientation of (+ , -) 35 degrees about the x-axis, which corresponds to looking towards the top of the table and looking towards the base of the table respectively. For example if the camera is at (0,2), + 35 degrees would make the camera look towards (0,3) and - 35 degrees would make the camera look towards (0,1)

Your task is to answer the question: {question}

    Object Identification: Locate the object in the scene that is relevant to the question. Identify the object's position within the grid and determine its coordinates as (vertex_x, vertex_y, vertex_z).

    Action Recommendation: First try to answer the question, {question}. 
    If you do not have an answer and the objects are not fully visible or are occluded, recommend a new camera position and orientation to answer the question, {question}. Use the object's position to recommend a new camera vertex and orientation using the cube's coordinate system. Note that you can use continuous values for vertex_x and vertex_y. For example (2.2,1.5)
    The new camera position should be expressed as float values (vertex_x, vertex_y, vertex_z) rounded to one digit. The new camera orientation should be expressed as integer value (orient_x).
        orient_x = 0 for the camera looking perpendicularly down on the surface of the table.
        orient_x = 1 for + 35 degrees (looking towards the top of the table). 
        orient_x = 2 for - 35 degrees (looking towards the base of the table).
    The camera stops 35 cm above the selected vertex to avoid collisions. Explain your reasoning.
    Keep in mind that you may have to go further along the y-axis to look into an object. For example if the object is at (0,2) you may have to go to (0,3) and change orientation to -35 degrees to look inside an object.

    Comparison and Assertion: Compare the object's coordinates (vertex_x, vertex_y, vertex_z) with the recommended camera coordinates (vertex_x, vertex_y, vertex_z). Assert that the recommendation is correct by verifying that the new camera position provides a clear view of the object.

    Final Confirmation: Confirm whether you can now answer the question, {question}. Provide the answer.

Important: At the end of your response, If the question is answered, on a new line provide the output without any spaces as: 
output:("stop","stop","stop") 
If the question remains unanswered, on a new line provide the correct camera coordinates in the following format without any spaces: 
output:(vertex_x,vertex_y,vertex_z,orient_x)
"""

old_prompt = f"""
Analyze the provided image of a tabletop scene captured by a camera mounted on a UR5 robotic manipulator. The scene includes various objects and an augmented 3D cube with grid vertices 15 cm apart, where the origin (0,0,0) aligns with the Aruco marker ID 8 close to the base of the robot. Note any objects that appear partially or fully occluded. Use the grid structure to specify the coordinates and extent of any occlusion.

The cube's z-axis is divided into color-coded layers: 0 represents the blue layer (bottom-most on table's surface), 1 represents the black layer, and 2 represents the green layer (top-most). The camera can also provide views from a different orientation of (+ , -) 35 degrees about the x-axis, which corresponds to looking towards the top of the table and looking towards the base of the table respectively. For example if the camera is at (0,2), + 35 degrees would make the camera look towards (0,3) and - 35 degrees would make the camera look towards (0,1)

Your task is to answer the question: {question}

    Object Identification: Locate the object in the scene that is relevant to the question. Identify the object's position within the grid and determine its coordinates as (vertex_x, vertex_y, vertex_z).

    Action Recommendation: First try to answer the question, {question}. 
    If you do not have an answer and the objects are not fully visible or are occluded, recommend a new camera position and orientation to answer the question, {question}. Use the object's position to recommend a new camera vertex and orientation using the cube's coordinate system. Note that you can use continuous values for vertex_x and vertex_y. For example (2.2,1.5)
    The new camera position should be expressed as float values (vertex_x, vertex_y, vertex_z) rounded to one digit. The new camera orientation should be expressed as integer value (orient_x).
        orient_x = 0 for the camera looking perpendicularly down on the surface of the table.
        orient_x = 1 for + 35 degrees (looking towards the top of the table). 
        orient_x = 2 for - 35 degrees (looking towards the base of the table).
    The camera stops 35 cm above the selected vertex to avoid collisions. Explain your reasoning.
    Keep in mind that you may have to go further along the y-axis to look into an object. For example if the object is at (0,2) you may have to go to (0,3) and change orientation to -35 degrees to look inside an object.

    Comparison and Assertion: Compare the object's coordinates (vertex_x, vertex_y, vertex_z) with the recommended camera coordinates (vertex_x, vertex_y, vertex_z). Assert that the recommendation is correct by verifying that the new camera position provides a clear view of the object.

    Final Confirmation: Confirm whether you can now answer the question, {question}. Provide the answer.

Important: At the end of your response, If the question is answered, on a new line provide the output without any spaces as: 
output:("stop","stop","stop") 
If the question remains unanswered, on a new line provide the correct camera coordinates in the following format without any spaces: 
output:(vertex_x,vertex_y,vertex_z,orient_x)
"""

full_prompt = f"""
Analyze the provided image of a tabletop scene captured by a camera mounted on a UR5 robotic manipulator.

Input:

    Image of Scene: A captured image showing objects placed on a table, some of which may be occluded. The image is augmented with a virtual 3D cube with where vertices are spaced 15 cm apart. The cube represents the coordinate frame, and the origin (0, 0, 0) is marked by an Aruco marker with ID 8.  The first image is encoded as 'encoded_image1' and provides an overhead view from 80 cm above the surface of the table.
        The cube's z-axis is divided into three distinct color-coded layers:
            vertex_z = 0 (Blue layer) is at the table surface.
            vertex_z = 1 (Black layer) is above the table.
            vertex_z = 2 (Green layer) is the topmost layer above the table.

    Coordinate Frame: The vertex_x, vertex_y, and vertex_z values for the camera position are defined in this overlaid cube's coordinate frame. The robot can move the camera to these positions using its 6DOF. vertex_x and vertex_y can have continuous values rounded to one decimal place with maximum value 0f 3.0. vertex_z and is an integer value.

    Camera Orientation: The camera on the robot's wrist can tilt:
        orient_x = 0 for a top-down perpendicular view.
        oorient_x = 1 for a +35° upward tilt.  For example if the camera is at (0.5,2.0), orient_x = 1 would make the camera look towards (0.5,3.0) 
        orient_x = 2 for a -35° downward tilt.  For example if the camera is at (0.5,2.0), orient_x = 2 would make the camera look towards (0.5,1.0). 

    Important Camera Placement Note: If GPT-4o decides to change the camera's orientation (i.e., by tilting along the x-axis), it should ensure the camera is positioned a few centimeters away from the object of interest to maintain a clear view. Being directly above the object while tilting would make the camera look away from it. Therefore, GPT-4o must recommend positions slightly offset from the object for better visibility when tilting.
    Additionally, the robot will stop at 35 cm above a given vertex_z to avoid any risk of collision.

    Question: {question}

    Answer Criteria:
        If GPT-4o can answer the question based on the current image and the cube's spatial reference, it will analyze the image and provide an answer along with:
            output:("stop","stop","stop","stop")
        If GPT-4o cannot answer the question due to occlusion or poor visibility, it will recommend moving the camera to the next best position and orientation (based on the target object's position, height, and orientation) in the format:
            output:(vertex_x,vertex_y,vertex_z,orient_x)
            The new camera position must allow for a better view to answer the question.
        Important: Provide the output without any spaces

Example:

    Question: "Is the mug full?"
    GPT-4o Response:
        Analysis: The mug is clearly visible in the current image.
        Answer: Yes, the mug is full. There is coffee in the mug.
        output:("stop","stop","stop","stop")

Example (Occlusion Case):

    Question: "Is the mug full?"
    GPT-4o Response:
        Analysis: The mug is partially occluded by another object in the current image, making it hard to see its contents.
        Recommended Camera Position: Move the camera to a better viewpoint to observe the mug's contents. 
        output:(vertex_x,vertex_y,vertex_z,orient_x)
"""

prompt = f"""
Analyze the provided image of a tabletop scene captured by a camera mounted on a UR5 robotic manipulator.

Input:

    Image of Scene: A captured image showing objects placed on a table, some of which may be occluded. The image is augmented with a virtual 3D cube where vertices are spaced 15 cm apart. The cube represents the coordinate frame, and the origin (0, 0, 0) is marked by an Aruco marker with ID 8. 
        The cube's z-axis is divided into three distinct color-coded layers:
            vertex_z = 0 (Blue layer) is at the table surface.
            vertex_z = 1 (Black layer) is above the table.
            vertex_z = 2 (Green layer) is the topmost layer above the table.

    Coordinate Frame: The vertex_x, vertex_y, and vertex_z values for the camera position are defined in this overlaid cube's coordinate frame. The robot can move the camera to these positions using its 6DOF. vertex_x and vertex_y can have continuous values rounded to one decimal place with maximum value 0f 3.0. vertex_z is an integer value.

    Camera Orientation: The camera on the robot's wrist can tilt:
        orient_x = 0 for a top-down perpendicular view.
        orient_x = 1 for a +35° upward tilt.  For example if the camera is at (0.5,2.5), orient_x = 1 would make the camera look towards (0.5,3.0) 
        orient_x = 2 for a -35° downward tilt.  For example if the camera is at (0.5,2.5), orient_x = 2 would make the camera look towards (0.5,1.0).
   
    Important Camera Placement Note: If GPT-4o decides to change the camera's orientation (i.e., by tilting along the x-axis), it should ensure the camera is positioned a few centimeters away from the object of interest to maintain a clear view. Being directly above the object while tilting would make the camera look away from it. Therefore, GPT-4o must recommend positions slightly offset from the object for better visibility when tilting.
    Additionally, the robot will stop at 35 cm above a given vertex_z to avoid any risk of collision.

    Question: {question}

    Answer Criteria:
        If GPT-4o can answer the question based on the current image and the cube's spatial reference, it will analyze the image and provide an answer along with:
            output:("stop","stop","stop","stop")
        If GPT-4o cannot answer the question due to occlusion or poor visibility, it will recommend moving the camera to the next best position and orientation (based on the target object's position, height, and orientation) in the format:
            output:(vertex_x,vertex_y,vertex_z,orient_x)
            The new camera position must allow for a better view to answer the question.
        Important: Provide the output without any spaces

Example:

    Question: "Is the mug full?"
    GPT-4o Response:
        Analysis: The target object is mug. The mug is clearly visible in the current image.
        Answer: Yes, the mug is full. There is coffee in the mug.
        output:("stop","stop","stop","stop")

Example (Occlusion Case):

    Question: "Is the mug full?"
    GPT-4o Response:
        Analysis: The mug is partially occluded by another object in the current image, making it hard to see its contents.
        Recommended Camera Position: Move the camera to a better viewpoint to observe the mug's contents.
        output:(vertex_x,vertex_y,vertex_z,orient_x)
"""

initial_prompt = f"""
You are an intelligent embodied agent that can perfrom active perception using visual and textual information. Your job is to analyze the provided image of a tabletop scene given an input text question. 

The input image will have a 3D cube with 3x3 grids overlaid on it. the grid's x and y vertices will be annotated in the format [x; y]. You must use the below conventions while proving the ouput

Base of tbale: Lower limit of 3D cube's x-axis i.e (0,0 to 3,0)
Top of table: Upper limit of 3D cube's x-axis i.e (0,3 to 3,3)

Layer Mapping: The 3D cube's z-axis consists of 3 discrete layers that are color coded. They represent the depth.
    - vertex_z = 0 (Blue layer) is at the table surface.
    - vertex_z = 1 (Black layer) is above the table.
    - vertex_z = 2 (Green layer) is the topmost layer above the table.
    
Orientation Mapping: This is the fixed orientation of 35 degrees about x-axis (roll in our case). Note that (0,0) is at the base of the table and (0,3) is at the top of the table:
    - orient_x = 0 when the target object of interest is flat on the table and perpendicular to the table 
    - orient_x = 1 for a +35° roll. Target is inclined towards the Base of Table
    - orient_x = 2 for a -35° roll. Target is inclined towards the Top of Table

Upon analysing the input image and text question you are to provide


------------------------------------------------

Staring below, you should follow this format for output:

Target object: The target object of interest to answer the question
Thought: Your thinking as to what the target object and its location and orientation are and why 
The target object location: An intial estimate of the target object location
    - vertex_x: Positive continuous value along x-axis in the cube coordinate frame rounded to one decimal
    - vertex_y: Positive continuous value along y-axis in the cube coordinate frame rounded to one decimal
    - vertex_z: Integer value along z-axis based on the Layer Mapping
    - orient_x: Integer value based on Orientation Mapping


---------------------------------------------

Text question: {question}
"""




# print(full_prompt)
generate_result()
