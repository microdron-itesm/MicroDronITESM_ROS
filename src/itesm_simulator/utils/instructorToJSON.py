"""
Este programa ayuda para la lectura de archivos .json
Se usa principalmente para poder pasar de un archivo .csv a un .json
"""
import json

def readInstructions() -> list:
    f = open("visualize.csv","r")
    instructions = f.readlines()
    f.close()
    result = []
    for x in instructions:
        result.append(x.rstrip())
    return result

if __name__ == "__main__":
    i = readInstructions()
    d = {
        "coordination" : 0,
        "instructions" : []
    }
    for x in range(len(i)):
        d["instructions"].append(i[x])
    j = json.dumps(d)
    j = j.replace(" ","")
    with open("directions.json","w") as f:
        f.write(j)
        f.close()
    # qr = createQR(json.dumps(d))
    # qr.save("stuff.png")
    