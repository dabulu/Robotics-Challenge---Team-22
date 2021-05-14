#variable file for masks of the colours
import cv2


# ----------
# thresholds
# ----------

#--- turquoise ---
turquoise_lt = (85, 130, 100) # lower threshold
turquoise_ut = (95, 255, 255) # upper threshold
turquoise_thresholds = [turquoise_lt, turquoise_ut]

#--- red ---
#red hue goes across axis so needs two masks and therefore two sets of thresholds
red_lt1 = (0, 180, 100)
red_ut1 = (10, 255, 255)
red_thresholds1 = [red_lt1, red_ut1]
red_lt2 = (170, 180, 100)
red_ut2 = (180, 255, 255)
red_thresholds2 = [red_lt2, red_ut2]
red_thresholds = [red_thresholds1, red_thresholds2]

#--- green ---
green_lt = (55, 120, 100)
green_ut = (65, 255, 255)
green_thresholds = [green_lt, green_ut]

#--- yellow ---
yellow_lt = (25, 125, 100)
yellow_ut = (35, 255, 255)
yellow_thresholds = [yellow_lt, yellow_ut]

#--- blue ---
blue_lt = (115, 225, 100)
blue_ut = (130, 255, 255)
blue_thresholds = [blue_lt, blue_ut]

#--- purple ---
purple_lt = (145, 150, 100)
purple_ut = (155, 255, 255)
purple_thresholds = [purple_lt, purple_ut]

colour_thresholds = [turquoise_thresholds, red_thresholds, green_thresholds,
    yellow_thresholds, purple_thresholds, blue_thresholds]

def getColour(val):

    colours = ["turquoise", "red", "green", "yellow", "purple", "blue"]
    return colours[val]

    """
    returns string name of colour from array
    """
    # colours = ["turquoise", "red", "green", "yellow", "blue", "purple"]



def getMask(img, colour):
    """
    generates a mask from an image paramater and a colour param
    colour: range of 0 to 5, each integer represents a colour
    [turquoise, red, green, yellow, blue, purple]
    [ 0  ,  1 ,   2  ,   3   ,  4  ,    5   ]
    """
    if colour == 1:
        red_t = colour_thresholds[colour]
        mask1 = cv2.inRange(img, red_t[0][0], red_t[0][1])
        mask2 = cv2.inRange(img, red_t[1][0], red_t[1][1])
        mask = mask1 + mask2
    else:
        if colour > 5 or colour < 0:
            print("Invalid colour integer, input 0 to 5")
        mask = cv2.inRange(img, colour_thresholds[colour][0], colour_thresholds[colour][1])
    return mask

def checkMaskOutputWithColour(img, colour):
    """
    Check that a mask has an effect on the input image using colour input
    """
    return 255 in getMask(img, colour)

def checkMaskOutput(mask):
    """
    check mask has an effect
    """
    return 255 in mask

def determineColour(img):
    """
    return colour whose mask has an effect
    """
    for i in range(0,6):
        if checkMaskOutputWithColour(img, i):
            return i
