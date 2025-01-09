import json
import os

json_file_path = 'settings.json'

def update_settings(height, width):
    data = {
        'height': height,
        'width': width
    }
    with open(json_file_path, 'w') as json_file:
        json.dump(data, json_file, indent=4)

def load_settings():
    if not os.path.exists(json_file_path):
        print("File Does Not Exist")
    
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)
    
    return data
