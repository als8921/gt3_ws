import json
import os

json_file_path = '/home/lmc/gt3_ws/settings.json'

def update_setting(setting_name, key, value):
    """지정된 설정의 키를 새로운 값으로 업데이트합니다.

    Args:
        setting_name (str): 업데이트할 설정의 이름.
        key (str): 설정 내에서 업데이트할 키.
        value: 설정할 새로운 값.

    Raises:
        KeyError: 설정이나 키가 존재하지 않을 경우.
    """
    data = load_settings()

    if setting_name not in data:
        raise KeyError(f"설정 '{setting_name}'이(가) 존재하지 않습니다.")

    if key not in data[setting_name]:
        raise KeyError(f"키 '{key}'가 설정 '{setting_name}'에 존재하지 않습니다.")

    data[setting_name][key] = value

    with open(json_file_path, 'w') as json_file:
        json.dump(data, json_file, indent=4)

def load_settings():
    """JSON 파일에서 설정을 로드합니다.

    Returns:
        dict: JSON 파일에서 로드된 데이터.

    Raises:
        FileNotFoundError: JSON 파일이 존재하지 않을 경우.
    """
    if not os.path.exists(json_file_path):
        raise FileNotFoundError("파일이 존재하지 않습니다.")
    
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)
    
    return data

def load_setting(key, value):
    """설정에서 특정 키에 대한 값을 로드합니다.

    Args:
        key (str): 설정에서 찾을 키.
        value (str): 검색할 특정 값.

    Returns:
        키와 관련된 값을 반환합니다.

    Raises:
        KeyError: 키가 존재하지 않을 경우.
        FileNotFoundError: JSON 파일이 존재하지 않을 경우.
    """
    if not os.path.exists(json_file_path):
        raise FileNotFoundError("파일이 존재하지 않습니다.")
    
    with open(json_file_path, 'r') as json_file:
        data = json.load(json_file)
    
    return data[key][value]

if __name__ == "__main__":
    print(load_settings())
    print(load_setting("paint", "height"))
    update_setting("paint", "height", 0.1)
    print(load_setting("paint", "height"))