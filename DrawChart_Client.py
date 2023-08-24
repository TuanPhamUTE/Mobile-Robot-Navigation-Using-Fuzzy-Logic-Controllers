import matplotlib.pyplot as plt
import socket 
import math

HEADER = 64
PORT = 5791
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"
SERVER = "192.168.1.102"
ADDR1 = (SERVER, PORT)
client1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client1.connect(ADDR1)

# Send1 to server
def send1(msg):
    message = msg.encode(FORMAT)
    msg_length = len(message)
    send_length = str(msg_length).encode(FORMAT)
    send_length += b' '*(HEADER - len(send_length))
    client1.send(send_length)
    client1.send(message)
    print(client1.recv(2048).decode(FORMAT))


# Khởi tạo danh sách tọa độ để lưu quãng đường robot đã đi
path = []

# Biến lưu trữ đường mũi tên hiện tại
arrow_line = None
dir_line = None

# Khởi tạo tọa độ ban đầu của robot
x = 0
y = 0

xd = 300
yd = 180

# Vẽ hình tròn để đại diện cho robot
circle = plt.Circle((x, y), 4, color='green')
# circle_target = plt.Circle((xd, yd), 5, color='red')
# cross1, = plt.plot([xd - 5, xd + 5], [yd - 5, yd + 5], color='red')
# cross2, = plt.plot([xd - 5, xd + 5], [yd + 5, yd - 5], color='red')


# Khởi tạo đồ thị với kích thước lớn
fig, ax = plt.subplots(figsize=(5,5))
ax.set_aspect('equal')
# Thêm hình tròn vào đồ thị
ax.add_patch(circle)
# ax.add_patch(circle_target)
# ax.add_line(cross1)
# ax.add_line(cross2)
# Vẽ dấu "x" tại tọa độ (xd, yd)
ax.scatter(xd, yd, marker='x', color='red')

# Đặt giới hạn trục x và y
ax.set_xlim(-10, 310)
ax.set_ylim(-10, 190)

# Hiển thị lưới
ax.grid(False)



plt.show(block=False)

# Vòng lặp nhập giá trị x từ người dùng và cập nhật tọa độ của robot
while True:
	data = client1.recv(2048).decode(FORMAT)
	if data:
		plt.show(block=False)
		if arrow_line is not None:
			arrow_line.remove()  # Xóa đường mũi tên trước đó

		print(f"Received data from server: {data}")
		data = data.strip()  # Loại bỏ ký tự xuống dòng (\r\n)
		position = data.split(',')

		xn = None
		yn = None

		print(position)
		if len(position) >= 3:
			xn_str = position[0].strip("b''").strip("'")
			yn_str = position[1].strip()
			if xn_str and yn_str:
				xn = float(xn_str)
				yn = float(yn_str)
   
		if xn is not None and yn is not None:
			circle.center = (xn, yn)
			# Lưu tọa độ vào danh sách quãng đường
			path.append((xn, yn))
			plt.plot(*zip(*path), color='purple')  # Vẽ đường đi
			# arrow_line = plt.arrow(xn, yn, xd-xn, yd-yn, color='purple', linestyle='dotted', linewidth=0.5, head_width=5, head_length=8)
			
   			# Tính toán tọa độ đỉnh mũi tên
			arrow_x = xd+2
			arrow_y = yd+2

			# Vẽ đường mũi tên từ robot đến mục tiêu
			arrow_line = plt.annotate('', xy=(arrow_x, arrow_y), xytext=(xn, yn),arrowprops=dict(arrowstyle='->', color='blue'))
   
			plt.show(block=False)
			plt.show(block=False)
			plt.draw()

	plt.pause(1.5)  # Tạm dừng 0.6 giây để đồ thị hiển thị và cập nhật
plt.show()

    
