import serial

s = serial.Serial('COM3')
fo = open("adatok.csv", "w")

fo.write("Hőmérséklet;Páratartalom\n")

buff = []

while(1):
	#fo.write(s.read())
	buff+=s.read()
	#print(str(buff),end="\n\n\n")

	if buff[-1]==10:
		for i in range(len(buff)-1):
			fo.write(chr(buff[i]))
		buff=[]
		print("ping")