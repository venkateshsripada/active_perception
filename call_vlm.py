import openai
import base64
from openai import OpenAI


# GPT_4okey = 'sk-proj-Qt5Xmbq40CEgfTU0wMVgT3BlbkFJi9JjJ140dihGT0a0LGT8'
GPT_4okey = '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'
client = OpenAI(api_key=GPT_4okey)


# Function to encode the image
def encode_image(image_path):
  with open(image_path, "rb") as image_file:
    return base64.b64encode(image_file.read()).decode('utf-8')

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
                            "detail": "low"
                            },
                    },
            ],
        }
    ],
    max_tokens = 4000,
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
        

    
def get_prompt(history_data, coords):
    prompt_27_8 = f"""
        You are an intelligent agent that can analyse a given input image and text question and provide accurate vertex positions (x,y,z). The camera taking the image is attached to an end effector of a 6DOF robotic arm. 
        You are performing active perception (vision), where you identify the target object to perceive based on the input text question. The camera will move to whatever vertex you specify and be your "eyes". Hence provide the vertices at which you want to go to and gain additional visual information and the camera will reach there.
        Note that you have a 3x3 grid and the annotated vertices on the image go from (0,0) to (3,3). So, for example you can reach vertex (2,0) though it is not visible in your current view.

        The input image consists of an RGB image of a tabletop scene overlaid with 3D grids. The vertices of the 3x3 grids are annotated in the input image. The z-axis is out of plane denoted by the coloured layers. The blue, black, red, and green layers are denoted by (0,1,2,3) respectively. They represent the z-axis from bottom to top. The bottom most layer is 0.43m from the table and the uppermost layer is at 0.68m.

        The input text consists of a question that you need to answer. You will also be provided with your current vertex position (x,y,z). If your current vertex is ( , , ) that means you have an initial home position.

        You have a history of the vertices visited and a summary of what you have seen from there in the context. Call it "History". It will be empty if no vertex has been visited yet.

        The input text question asks you steps you need to take to move the robot's end effector so that the desired output is produced. You are required to do 4 steps
        1. Summary: Clearly Summarise what vertex you are currently in and what you currently see.
        2. Thought: Analyse the question and do a chain-of-thought to get an answer. 
        3. Answer: Give me the answer to the input question. 
        4. Output: If you cannot answer the question, give me vertex position (x,y,z) without any spaces in between. Else, give me Answer (from step 3 above) again.
        
        It may be that you need not take any further steps. If you need further information to get an answer to my input question, then (x,y,z) will be integer values. Else, just output the answer to the question I asked in text.

        Here are three examples for context

        Example1:
        Input Image: Image with 3D layered grid and annotated vertices
        Input text question: What is <near the book>?
        Current vertex: You are at < , , >
        History:

        Summary: I see <part of a book, mug, and block of wood with a paper on it>.
        Thought: I want to be able to answer the question <near the book?>. The target object to investigate thoroughly is the <book>. From the History, I infer that I have not visited any vertices yet. Currently <The book> is at <(1,1,0)> and may be occluding another object. As objects may be <occluded by the book>, I need to go <on top of the book> to get a better view and answer the question. 
        Answer: The answer to your question <What is near the book> is: I <cannot> answer yet.
        Output: To go <on top of the book> and get a better view, go to <(1,1,3)> and look again. 



        Example2:
        Input Image: Image with 3D layered grid and annotated vertices
        Input text question: What is <in the pitcher>?
        Current vertex: You are at <2,1,3>
        History: At vertex < , ,>. I see <the base of a pitcher and book>.

        Summary: At vertex <2,1,3>. I see a <the full pitcher, part of a book, and ball at vertex (0,1,0)>
        Thought: I want to be able to answer the question <what is in the pitcher>?. The target object to investigate thoroughly is the <pitcher>. From the history I infer that I had an initial view and detected the <base of the pitcher>. Currently,  I am at vertex <(2,1,3)> and do not completely see inside the pitcher. To be able to answer the question < what is in the pitcher>? I have to <go closer to the pitcher> to be able to <look inside it>. 
        Answer: The answer to your question <What is in the pitcher> is: I <cannot> answer yet
        Output: To <go closer to the pitcher> and <look inside it> go to <(2,3,3)>.



        Example3:
        Input Image: Image with 3D layered grid and annotated vertices
        Input text question: Is <there is an object behind the laptop>?
        Current vertex: You are at <1,1,2>
        History: At vertex < , , >. I see <laptop screen, spiral book>.
        At vertex <(3,0,3)>. I see <the right side of a laptop, part of a spiral book, phone, and part of a black mouse>.

        Summary: At vertex <(1,1,2)>. I see < the left side of the laptop, part of a book, and mouse>.
        Thought: I want to be able to answer the question <is there is an object behind the laptop>?. The target object to investigate thoroughly is the <laptop>. From the history I infer that I had an intial view and was at vertices <(3,0,3)>. I detected 4 objects <laptop, book, phone, and mouse>. Currently,  I am at vertex <(1,1,2)> and have been all around the laptop. I see the same 4 objects i.e <laptop, book, phone, and mouse>. I can confirm from my history and current states that I looked all around the <laptop>. I am able to answer <is there is an object behind the laptop>? 
        Answer: The answer to your question <Is there is an object behind the laptop> is: <Yes. There is a phone beside the laptop>
        Output: The answer to your question <Is there is an object behind the laptop> is: <Yes. There is a phone beside the laptop>


        Input image:
        Input text question: What coloured object is in coffee cup?
        Current Vertex: You are at ({coords[0]},{coords[1]},{coords[2]})
        History: {history_data}. 
        GPT-4o: 
        
        """

    return prompt_27_8





def generate_result():
    encoded_image1 = encode_image(image_path + "grid_image.jpg")

    # Get only the Summary from gpt_log
    summary = get_summary_section(gpt_result_path)
    coord_array = get_vertex_position(gpt_result_path)
    # if not coord_x:


    
    # Add summary to prompt history context
    if summary != None:
        with open(prompt_history_path, "a") as file:
            file.write("\n")
            file.write(summary)
    else:
        with open(prompt_history_path, "w") as file:
            print("New prompt history log file")

    # # Get iterated prompt + history
    # try:
    with open(prompt_history_path, "r") as history_file:
        history_data = history_file.read()  
        full_prompt = get_prompt(history_data, coord_array)
    # except FileNotFoundError:
    #     with open(prompt_history_path, "w") as history_file:          
    #         print("Created new file")
    

    # Give input to VLM
    result = analyze_image(model, encoded_image1, full_prompt)
    # print(result)

    # Add result to a file
    with open(gpt_result_path, "w") as file:
        file.write(result)

    # Append result to log file
    with open(gpt_log_path, "a") as file:
        file.write("\n \n -------Next time step ------------------ \n \n")
        file.write(result)

    # Append prompt to log file
    with open(prompt_log_path, "a") as file:
        file.write("\n \n -------Next time step ------------------ \n \n")
        file.write(full_prompt)


model = "gpt-4o"
image_path = "/home/ur_5/ur5_ws/"

trial = 5
prompt_history_path = f"/home/ur_5/Documents/gpt_tests/Test_29_8_4/prompt_history_{trial}.txt"
prompt_log_path = f"/home/ur_5/Documents/gpt_tests/Test_29_8_4/prompt_log_{trial}.txt"
gpt_result_path = f"/home/ur_5/Documents/gpt_tests/Test_29_8_4/gpt_result_{trial}.txt"
gpt_log_path = f"/home/ur_5/Documents/gpt_tests/Test_29_8_4/gpt_log_{trial}.txt"

generate_result()



prompt = """

You are an intelligent agent that can provide accurate vertex positions (x, y, z) given an input image and an input text question. The camera taking the image is attached to an end effector of a 6DOF robotic arm.

The input image consists of an RGB image of a tabletop scene overlaid with 3D grids. The x-axis is from left to right of the image and the y-axis is from top to bottom of the image. The z-axis is out of plane denoted by the coloured layers. The blue, black, red, and green layers are denoted by (0,1,2,3) respectively. They represent the z-axis from bottom to top. The bottom most layer is 0.22m from the table and the uppermost layer is at 0.82m. Each layer is separated by approximately 0.20m

Each dot represents a vertex of the 3D grid. The vertex to the left end of the image is 0 and to the right end is 3. So, there are 4 vertices in each layer along the x and y axis. For example the bottom left vertex is (0, 0, 0), 
the vertex to its right is (1,0, 0),
the vertex above it is (1,1, 0), 
and the corner most vertex is (3,3, 0).
Similarly 
the top most left vertex is (0, 0,3), 
the vertex to its right is (1,0, 3),
the vertex above it is (1,1, 3), 
and the corner most vertex is (3,3, 3).

The input text question asks you steps you need to take to move the robot's end effector so that the desired output is produced. It may be that you need not take any furher steps. So analyse well.

As output I want you to think about what the answer to the input question is and give me the vertex position i.e (x,y,z). Give me vertex position (x,y,z) without any spaces in between. If you need further information to get an answer to my input question, then x,y,z will be integer values. Else, x,y,z will be strings "stop", "stop", "stop" 

Here are a couple of examples for context

Input Image: Image with 3D layered grid and non-annotated red vertices
Input text: What is <near the book>?
Output: The best way to know if <there is something near the book>  is to go to <(2,3,3)> and look again. This is so that <we get an unoccluded view of the objects near the book>

Input Image: Image with 3D layered grid and non-annotated red vertices
Input text: Is <there is an object behind the laptop>?
Output: The best way to know if <there is an object behind the laptop>  is to go to <(1,2,2)> and look again. This is so that <we can see objects behind the laptop>

Input Image: Image with 3D layered grid and non-annotated red vertices
Input text: Is <there is a pen beside the book>?
Output: The best way to know if  <there is a pen beside the book>  is to stay where I am as my current view is the best view. Hence (stop,stop,stop) robot's end effector motion. The answer to your question of Is <there is a pen beside the book> is <yes there is a pen>.

Input Image: Image with 3D layered grid and non-annotated red vertices
Input text: What is <near the bottle>?
Output: The best way to know What is <near the bottle>? is to stay where I am as my current view is the best view. Hence (stop,stop,stop) robot's end effector motion. The answer to your question of What is <near the bottle>? is <a cap>.

Input image:
Input text: What is in the blue cup?
Output:"""