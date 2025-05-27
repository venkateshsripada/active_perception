
scene = 1
trial = 1
scene_name = "AP_VLM"
base_path = "/media/venkatesh/Storage/venkatesh/IROS_data"

model = "gpt-4o"
GPT_4okey = ''

log_data_base_path = "/media/venkatesh/Storage"



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
