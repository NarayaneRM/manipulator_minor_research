def max_t1_speed(my_client):
    my_client.send_command('setJointAcceleration 1.0')
    my_client.send_command('setJointVelocity 0.5')
    my_client.send_command('setJointJerk 1.0')
    my_client.send_command('setCartVelocity 10000')


def max_t2_speed(my_client):
    my_client.send_command('setJointAcceleration 0.3')
    my_client.send_command('setJointVelocity 1.0')
    my_client.send_command('setJointJerk 0.1')
    my_client.send_command('setCartVelocity 10000')
    
    
def max_auto_speed(my_client):
    my_client.send_command('setJointAcceleration 1.0')
    my_client.send_command('setJointVelocity 1.0')
    my_client.send_command('setJointJerk 0.1')
    my_client.send_command('setCartVelocity 10000')
    

def medium_auto_speed(my_client):
    my_client.send_command('setJointAcceleration 0.7')
    my_client.send_command('setJointVelocity 0.7')
    my_client.send_command('setJointJerk 0.1')
    my_client.send_command('setCartVelocity 7000')


def slow_auto_speed(my_client):
    my_client.send_command('setJointAcceleration 0.4')
    my_client.send_command('setJointVelocity 0.4')
    my_client.send_command('setJointJerk 0.1')
    my_client.send_command('setCartVelocity 4000')
    