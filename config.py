
scene = 2
trial = 11
scene_name = "AP_VLM"
base_path = "/media/venkatesh/Storage/venkatesh/IROS_data"

model = "gpt-4o"
GPT_4okey = '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'

log_data_base_path = "/media/venkatesh/Storage"

interactive_perception = True
#--- for  interactive perception----
if interactive_perception:
    scene_name = "RSS_data/PIVOT"
    base_path = "/home/venkatesh/Documents/interactive_perception_tests"
    log_data_base_path = "/media/venkatesh/Storage/RSS_logged_data_video"
    grasp = True
    push = False
#-------------------------------------------

image_save_path = "{}/{}/scene_{}".format(base_path, scene_name, scene)
video_save_path = "{}/{}/scene_{}/trial_{}/".format(base_path, scene_name, scene, trial)
result_path = "{}/{}/scene_{}/trial_{}/gpt_result_{}.txt".format(base_path, scene_name, scene,  trial, trial)
gpt_analyser_path = "{}/{}/scene_{}/trial_{}/gpt_analyser_{}.txt".format(base_path, scene_name, scene,  trial, trial)

prompt_log_path = "{}/{}/scene_{}/trial_{}/prompt_log_{}.txt".format(base_path, scene_name, scene, trial, trial)

gpt_log_path = "{}/{}/scene_{}/trial_{}/gpt_log_{}.txt".format(base_path, scene_name, scene, trial, trial)

image_path = "{}/{}/scene_{}/trial_{}/".format(base_path, scene_name, scene, trial)
prompt_history_path = "{}/{}/scene_{}/trial_{}/prompt_history_{}.txt".format(base_path, scene_name, scene, trial, trial)

current_position_path = "{}/{}/scene_{}/trial_{}/current_position_{}.txt".format(base_path, scene_name, scene, trial, trial)
target_state_path = "{}/{}/scene_{}/trial_{}/target_position_{}.txt".format(base_path, scene_name, scene, trial, trial)
# result_path = "/home/ur_5/Documents/gpt_tests/Test_4_9_24/test_24/gpt_result_24.txt"
memory_dictionary_path = "{}/{}/scene_{}/trial_{}/".format(base_path, scene_name, scene, trial)

use_gpt = True
use_gemini = False
use_claude = False

# New Directory to log results
test_directory = "trial_{}".format(trial)

# Data collection questions
# For scene 1 
# question = "How many paper clips does this box contain?"

# For scene 2
# question = "What diameter nuts do I need for the two type of bolts?"

# For scene 3
# question = "What is in the coffee mug?"
# question = "What is inside the red mug?" 

# For scene 4
# question = "What is the title of the computer science textbook on the table?"

# For scene 5
# question = "What object is under the white cloth?"

# For Scene 6
# question = "To which engineering department should this package be sent, based on its contents?"

# For Scene 7
# question = "What brand is the measuring tape on the table?"

# For Scene 8 - Similar to question 3 (scene 3) but there is a tea bag inside the mug
# question = "What is inside the red mug?" 

# Scene 9
# question = "Is the bookshelf arranged properly? Can you identify what looks out of place in the bookshelf setup?"
# question = "Which shelf in this bookshelf should a wrench be placed? If your view isn't detailed enough, please move closer to inspect the shelf details before answering."



# question = "What specific type of cable is this?"
# question = "What brand is the measuring tape on the table?"
# question = "What is the reading on the 'Marathon' stop watch?"
# question = "What is the expiration date on this SPAM tin?"

#-------------------------------
if interactive_perception:
    question = "What text is written under the blue block?"
    # question =  "What language is the text written in on the black eraser?"
# question = "What is inside the orange cup?"
