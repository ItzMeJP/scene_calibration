#!/usr/bin/env python
import rospy
import tf
import numpy as np
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import time


def quaternion_normalise(q):
    norm = np.linalg.norm(q)
    if norm == 0:
        return q
    return q / norm


def publish_random_tf(parent_frame, child_frame):
    # Inicializar o broadcaster de TF
    br = TransformBroadcaster()

    # Valores originais da transformação
    original_position = [1.0, 2.0, 3.0]  # Exemplo: posição original
    original_orientation = [0.0, 0.0, 0.0, 1.0]  # Exemplo: orientação original (quaternion)

    # Variância permitida em torno dos valores originais
    variance = 0.05

    rate = rospy.Rate(4)  # 4Hz corresponde a 250ms

    while not rospy.is_shutdown():
        # Criar uma mensagem de transformação
        t = TransformStamped()

        # Preencher os campos da mensagem com valores aleatórios próximos dos originais
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = np.random.normal(original_position[0], variance)
        t.transform.translation.y = np.random.normal(original_position[1], variance)
        t.transform.translation.z = np.random.normal(original_position[2], variance)
        # Gerar um quaternion aleatório próximo do original
        random_quaternion = np.random.normal(original_orientation, variance)
        random_quaternion = quaternion_normalise(random_quaternion)
        t.transform.rotation = Quaternion(*random_quaternion)


        # Publicar a transformação
        br.sendTransform(t)

        # Esperar 250ms antes da próxima atualização
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('random_tf_publisher')

        # Obter os nomes dos frames do servidor de parâmetros
        parent_frame = rospy.get_param('~parent_frame', 'base_link')
        child_frame = rospy.get_param('~child_frame', 'child_frame')

        publish_random_tf(parent_frame, child_frame)
    except rospy.ROSInterruptException:
        pass
