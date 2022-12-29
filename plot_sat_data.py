import matplotlib.pyplot as plt


x_vals = []
pos_x_vals = []
pos_y_vals = []
pos_z_vals = []

time = 0.0
with open("./SatelliteSimulator/position_ECI.txt", "r") as fp:
   line = fp.readline()
   while line:
        nums = line.replace('\n', '').split(' ')
        if (len(nums) != 3):
            break

        
        pos_x_vals.append(float(nums[0]))
        pos_y_vals.append(float(nums[1]))
        pos_z_vals.append(float(nums[2]))
        
        
        x_vals.append(time)
        time += 0.1
        line = fp.readline()


plt.plot(x_vals, pos_x_vals, label='x')
plt.plot(x_vals, pos_y_vals, label='y')
plt.plot(x_vals, pos_z_vals, label='z')
plt.legend()
plt.show()