import matplotlib.pyplot as plt
x = [1, 2, 3, 4, 5]
y = [25, 32, 34, 20, 25]
plt.subplot(2,1,1)
plt.plot(x,y)
plt.xlabel('Ось х') #Подпись для оси х
plt.ylabel('Ось y') #Подпись для оси y
plt.title('Первый график') #Название
x = [1, 2, 3, 4, 5]
y = [70, 32, 30, 20, 20]
plt.subplot(2,1,2)
plt.plot(x, y)
plt.xlabel('Ось х') #Подпись для оси х
plt.ylabel('Ось y') #Подпись для оси y
plt.title('Первый график') #Название
plt.show()