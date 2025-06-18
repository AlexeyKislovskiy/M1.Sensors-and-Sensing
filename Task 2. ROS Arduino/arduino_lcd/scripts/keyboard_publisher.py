#!/usr/bin/env python3
import rospy
from std_msgs.msg import Char

def keyboard_command_node():
    rospy.init_node('keyboard_command_node', anonymous=True)
    pub = rospy.Publisher('/computer/lcd/cmd', Char, queue_size=10)
    
    print("Введите команды (W/A/S/D) для перемещения точки. Остальные символы игнорируются.")
    
    while not rospy.is_shutdown():
        try:
            # Считываем ввод пользователя
            cmd = input().upper().strip()
            for char in cmd:
                if char in ['W', 'A', 'S', 'D']:
                    rospy.loginfo(f"Отправка команды: {char}")
                    pub.publish(Char(ord(char)))
                else:
                    rospy.logwarn(f"Некорректная команда: {char}")
        except EOFError:
            break

if __name__ == '__main__':
    try:
        keyboard_command_node()
    except rospy.ROSInterruptException:
        pass