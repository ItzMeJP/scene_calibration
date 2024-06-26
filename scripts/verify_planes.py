#!/usr/bin/env python

import rospy
import numpy as np
import tf
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler
def publish_marker(plane_data, frame_id, _marker_id):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.CUBE
    marker.id = _marker_id
    marker.action = marker.ADD

    # Definindo a posição do marcador
    marker.pose.position.x = plane_data['p0']['x']
    marker.pose.position.y = plane_data['p0']['y']
    marker.pose.position.z = plane_data['p0']['z']

    # Calculando os vetores das arestas
    p0 = np.array([plane_data['p0']['x'], plane_data['p0']['y'], plane_data['p0']['z']])
    px = np.array([plane_data['px']['x'], plane_data['px']['y'], plane_data['px']['z']])
    py = np.array([plane_data['py']['x'], plane_data['py']['y'], plane_data['py']['z']])
    edge1 = px - p0
    edge2 = py - p0

    # Normalizando os vetores das arestas
    edge1 = edge1 / np.linalg.norm(edge1)
    edge2 = edge2 / np.linalg.norm(edge2)

    # Calculando a rotação para alinhar os eixos do marcador com os vetores das arestas
    rotation_matrix = np.array([edge1, edge2, np.cross(edge1, edge2)]).T

    # Convertendo a matriz de rotação 3x3 em uma matriz de transformação 4x4
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation_matrix

    quaternion = tf.transformations.quaternion_from_matrix(transformation_matrix)

    # Definindo a orientação do marcador
    marker.pose.orientation.x = quaternion[0]
    marker.pose.orientation.y = quaternion[1]
    marker.pose.orientation.z = quaternion[2]
    marker.pose.orientation.w = quaternion[3]

    # Definindo a escala do marcador (tornando-o grande o suficiente para parecer infinito)
    marker.scale.x = 10  # largura do plano
    marker.scale.y = 10  # altura do plano
    marker.scale.z = 0.01  # espessura do plano

    # Definindo a cor do marcador
    marker.color.a = 0.5  # semi-transparente
    marker.color.r = 1.0  # Canal vermelho
    marker.color.g = 0.0  # Canal verde
    marker.color.b = 0.0  # Canal azul

    # Publicando o marcador
    marker_pub = rospy.Publisher(f'visualization_marker_{_marker_id}', Marker, queue_size=10)
    marker_pub.publish(marker)

def main():
    rospy.init_node('plane_visualizer')

    # Lendo a string YAML do servidor de parâmetros
    data = rospy.get_param('/plane_data')

    # Criando um objeto Rate
    rate = rospy.Rate(10)  # 10 Hz

    # Publicando um marcador para cada plano em um loop
    while not rospy.is_shutdown():
        i=0
        for plane_id, plane_data in data['0_planes_workspace']['planes'].items():
            publish_marker(plane_data, data['0_planes_workspace']['ws_base_frame'], i)
            i=i+1
        rate.sleep()  # Dormir o suficiente para manter a taxa desejada

if __name__ == '__main__':
    main()
