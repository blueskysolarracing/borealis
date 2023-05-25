import os
def fix_lines(line_to_change)->str:

    this_file_path = os.path.abspath(__file__)
    this_file_path = this_file_path.replace("\\", "/")

    index = this_file_path.rfind("/Master_Workspace")
    if index == -1:
        print("Error: Please make sure your project is under Master_Workspace")
        quit()
    index = index + len("/Master_workspace")
    workspace_path = this_file_path[0:index].replace("c:/", "C:/")
    
    begin_idx = line_to_change.find("<location>")
    if begin_idx == -1:
        print("Error: <location> not found in file")
        quit()
    begin_idx = begin_idx + len("<location>")
    beginStr = line_to_change[0:begin_idx]

    replace_to_idx = line_to_change.find("/Shared_Resources")
    if replace_to_idx == -1:
        print("Error: Shared_Resources not found in line")
        quit()
    end_str = line_to_change[replace_to_idx:]

    new_line = beginStr + workspace_path + end_str
    print("Your updated path:")
    print(new_line)
    return new_line

def main():
    parent = os.path.dirname(os.path.abspath((__file__)))
    with open(os.path.join(parent, '.project'), 'r') as project_file:
        file_lines = project_file.readlines()
    with open(os.path.join(parent, '.project'), 'w') as project_file:
        for line in file_lines:
            if "/Shared_Resources" in line:
                project_file.write(fix_lines(line))
            else: 
                project_file.write(line)

if __name__ == "__main__":
    main()