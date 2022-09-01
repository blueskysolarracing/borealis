import os
def fixLine(lineToChange)->str:

    thisFilePath = os.path.abspath(__file__)
    thisFilePath = thisFilePath.replace("\\", "/")

    index = thisFilePath.rfind("/gen11_blueskyelec") + len("/gen11_blueskyelec")
    if index == -1:
        print("Error: Please make sure your top repository is gen11_blueskyelec")
        quit()

    topRepoPath = thisFilePath[0:index].replace("c:/", "C:/")
    
    beginIdx = lineToChange.find("<location>") + len("<location>")
    if beginIdx == -1:
        print("Error: <location> not found in file")
        quit()
    beginStr = lineToChange[0:beginIdx]

    replaceToIdx = lineToChange.find("/Master_Workspace")
    if replaceToIdx == -1:
        print("Error: /Master_Workspace not found in file")
        quit()
    endStr = lineToChange[replaceToIdx:]

    newLine = beginStr + topRepoPath + endStr
    print("Your updated path:")
    print(newLine)
    return newLine

def main():
    parent = os.path.dirname(os.path.abspath((__file__)))
    with open(os.path.join(parent, '.project'), 'r') as projectFile:
        fileLines = projectFile.readlines()
    with open(os.path.join(parent, '.project'), 'w') as projectFile:
        for line in fileLines:
            if "/Shared_Resources" in line:
                projectFile.write(fixLine(line))
            else: 
                projectFile.write(line)
if __name__ == "__main__":
    main()