import qrcode

def createQR(s : str):
    """
    Creates a PilImage from the parameter
    in the form of a QR.

    Args:
        s (str): String to be converte to QR

    Returns:
        PilImage: QR Image of the String passed as Param
    """
    return qrcode.make(s)

if __name__ == "__main__":
    info = ""
    with open("directions.json","r") as f:
        info = f.readline()
        f.close()
    qr = createQR(info)
    qr.save("result.png")