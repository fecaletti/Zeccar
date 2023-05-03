Import("env")

# In-line command with arguments
env.Replace(
    UPLOADCMD="stm32flash -w $SOURCE /dev/cu.usbserial-22310"
)