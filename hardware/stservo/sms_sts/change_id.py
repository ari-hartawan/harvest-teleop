#!/usr/bin/env python3
import sys
import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# --- PERBAIKAN IMPORT PATH ---
CURRENT_DIR = os.path.dirname(os.path.abspath(__file__)) # /home/user/tm_ws/TM5_teleop/stservo/sms_sts

# Kita cari letak folder 'TM5_teleop' secara dinamis
# Berdasarkan path Anda, TM5_teleop adalah folder yang berada 2 tingkat di atas file ini
TM5_TELEOP_DIR = os.path.abspath(os.path.join(CURRENT_DIR, '../..')) 

if TM5_TELEOP_DIR not in sys.path:
    sys.path.append(TM5_TELEOP_DIR)

try:
    from stservo.scservo_sdk.port_handler import PortHandler
    from stservo.scservo_sdk.sms_sts import sms_sts
    from stservo.scservo_sdk.scservo_def import *
except ImportError as e:
    print(f"Error detail: {e}")
    print(f"Python mencari di folder: {sys.path}")
    print("Error: SDK Feetech tidak ditemukan. Pastikan folder 'stservo' ada di path yang benar.")
    sys.exit(1)

# Default setting
SCS_ID = 1            # ID Lama Servo (Default Pabrik adalah 1)
NEW_ID = 2            # ID Baru yang Dinginkan
BAUDRATE = 1000000     
DEVICENAME = '/dev/ttyACM0'   

# Initialize PortHandler instance
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
packetHandler = sms_sts(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to set the baudrate")
else:
    print("Failed to set the baudrate")
    portHandler.closePort()
    quit()

print(f"Membuka kunci EPROM Servo ID {SCS_ID}...")
scs_comm_result, scs_error = packetHandler.unLockEprom(SCS_ID)
if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
    quit()
elif scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
    quit()

# --- PERBAIKAN REGISTER ADDRESS ---
# Alamat register untuk ID Servo pada ST3215 adalah 5 (SMS_STS_ID atau 0x05)
# Di kode Anda sebelumnya, ada variabel 'scs_id' yang tidak terdefinisi dan menyebabkan error tersembunyi
print(f"Mengubah ID dari {SCS_ID} menjadi {NEW_ID}...")
scs_comm_result, scs_error = packetHandler.write1ByteTxRx(SCS_ID, 5, NEW_ID)

if scs_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(scs_comm_result))
else:
    # Mengunci kembali EPROM setelah selesai mengubah ID
    packetHandler.LockEprom(NEW_ID) # Kunci menggunakan ID yang baru
    print(f"SUCCESS: Berhasil mengubah Servo ID menjadi {NEW_ID}!")

if scs_error != 0:
    print("%s" % packetHandler.getRxPacketError(scs_error))
    getch()
    quit()

# Menutup port kembali
portHandler.closePort()