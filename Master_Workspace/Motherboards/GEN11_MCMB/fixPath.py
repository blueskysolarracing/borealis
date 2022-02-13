def fixLine(lineToChange)->str:

    thisFilePath = __file__
    thisFilePath = thisFilePath.replace("\\", "/")

    index = thisFilePath.rfind("/gen11_blueskyelec") + len("/gen11_blueskyelec")

    topRepoPath = thisFilePath[0:index].replace("c:/", "C:/")
    
    beginIdx = lineToChange.find("<location>") + len("<location>")
    beginStr = lineToChange[0:beginIdx]
    print(lineToChange[0:beginIdx])

    replaceToIdx = lineToChange.find("/Master_Workspace")
    endStr = lineToChange[replaceToIdx:]
    print(endStr)

    newLine = beginStr + topRepoPath + endStr
    print(newLine)
    return newLine

def main():
    projectFile = open('.project', 'r')
    fileLines = projectFile.readlines()
    projectFile.close()
    projectFile = open('.project', 'w')
    i = 0;
    while i < len(fileLines):
        if "/Shared_Resources" in fileLines[i]:
            projectFile.write(fixLine(fileLines[i]))
        else: 
            projectFile.write(fileLines[i])
        i += 1;
if __name__ == "__main__":
    main()