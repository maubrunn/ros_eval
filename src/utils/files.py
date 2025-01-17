def get_text_of_last_dir(dir):
    if dir[-1] == "/":
        return dir.split("/")[-2]
    return dir.split("/")[-1]