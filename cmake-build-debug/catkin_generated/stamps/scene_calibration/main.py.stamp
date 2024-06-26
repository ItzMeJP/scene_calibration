#!/usr/bin/env python

import rospy, tf, yaml, rospkg
from enum import Enum
import numpy as np
from geometry_msgs.msg import TransformStamped
import time, os, sys, copy

class Method(Enum):
    SINGLE_FRAME = 0
    MULTI_FRAME = 1
    MULTI_PLANES = 2
    MULTI_PLANES_PRECISE = 3

def generate_planes_from_frames(_measurements, _plane_z_offset):
    planes = []
    for _, transform in enumerate(_measurements):
        p0 = copy.deepcopy(transform)
        px = copy.deepcopy(transform)
        py = copy.deepcopy(transform)

        # Create a translation transformation
        trans_px = tf.transformations.translation_matrix((1, 0, 0))
        trans_py = tf.transformations.translation_matrix((0, 1, 0))
        trans_pz = tf.transformations.translation_matrix((0, 0, _plane_z_offset))


        # Convert TransformStamped to 4x4 matrix
        trans = [transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z]
        rot = [transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w]
        mat = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans), tf.transformations.quaternion_matrix(rot))

        mat_pz = np.dot(mat, trans_pz)

        p0.transform.translation.x, p0.transform.translation.y, p0.transform.translation.z = tf.transformations.translation_from_matrix(mat_pz)

        # Convert TransformStamped to 4x4 matrix
        trans2 = [p0.transform.translation.x, p0.transform.translation.y, p0.transform.translation.z]
        rot2 = [p0.transform.rotation.x, p0.transform.rotation.y, p0.transform.rotation.z, p0.transform.rotation.w]
        mat2 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans2), tf.transformations.quaternion_matrix(rot2))

        # Apply the translation transformation
        mat_px = np.dot(mat2, trans_px)
        mat_py = np.dot(mat2, trans_py)

        # Convert the resulting matrix back to a TransformStamped
        px.transform.translation.x, px.transform.translation.y, px.transform.translation.z = tf.transformations.translation_from_matrix(mat_px)
        py.transform.translation.x, py.transform.translation.y, py.transform.translation.z = tf.transformations.translation_from_matrix(mat_py)

        planes.append(p0)
        planes.append(px)
        planes.append(py)
    return planes


def generateYamlDescription(_origin, _arr):

    for transform in _arr:
        if transform.header.frame_id != _origin.child_frame_id:
            print(f"Error while generating the yaml file: the parent_frame_id  {transform.header.frame_id} does not correspond to the origin child_frame_id.")
            return

    data = {
        '0_planes_workspace': {
            'ws_base_frame': _origin.child_frame_id,
            'approach_namespace': 'grasping_approach',
            'planes': {}
        }
    }

    for i in range(0, len(_arr), 3):
        plane_id = f"{i//3}_plane"
        data['0_planes_workspace']['planes'][plane_id] = {
            'p0': {'x': float(_arr[i].transform.translation.x), 'y': float(_arr[i].transform.translation.y), 'z': float(_arr[i].transform.translation.z)},
            'px': {'x': float(_arr[i+1].transform.translation.x), 'y': float(_arr[i+1].transform.translation.y), 'z': float(_arr[i+1].transform.translation.z)},
            'py': {'x': float(_arr[i+2].transform.translation.x), 'y': float(_arr[i+2].transform.translation.y), 'z': float(_arr[i+2].transform.translation.z)},
        }

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('scene_calibration')
    output_file_path = os.path.join(package_path, 'output', 'output.yaml')

    with open(output_file_path, 'w') as outfile:
        yaml.dump(data, outfile, default_flow_style=False)

    rospy.loginfo(f"Yaml exported to {package_path}/output.launch.")


def process_samples(_samples):

    samples_array = np.array(_samples)
    mean = np.mean(samples_array, axis=0)
    std_dev = np.std(samples_array, axis=0)
    filtered_samples = samples_array[np.all(np.abs(samples_array - mean) <= 2 * std_dev, axis=1)]
    mean_filtered_samples = np.mean(filtered_samples, axis=0)
    mean_filtered_samples_rounded = [round(num, 6) for num in mean_filtered_samples]

    return mean_filtered_samples_rounded


def getMeasurement(_tf_listener, _parent_frame, _child_frame, _num_samples, _delay_between_samples, _timeout):

    position_samples = []
    orientation_samples = []
    t = TransformStamped()

    try:
        rospy.loginfo(f"Waiting for relation TF {_child_frame} w.r.t {_parent_frame} to be available.")
        _tf_listener.waitForTransform(_parent_frame, _child_frame, rospy.Time(), rospy.Duration(_timeout))
        rospy.loginfo(f"TF {_child_frame} w.r.t {_parent_frame} is available!")
        rospy.loginfo(f"Acquiring, please wait...")
        for _ in range(_num_samples):
            try:
                (trans, rot) = _tf_listener.lookupTransform(_parent_frame, _child_frame, rospy.Time(0))
                position_samples.append(trans)
                orientation_samples.append(rot)
                time.sleep(_delay_between_samples)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Error looking TF. Ignoring measurement...")
    except rospy.ROSException:
        rospy.logerr("Error while waiting for TF. Time threshold exceeded.")

    processed_position = process_samples(position_samples)
    processed_orientation = process_samples(orientation_samples)

    t.transform.translation.x = processed_position[0]
    t.transform.translation.y = processed_position[1]
    t.transform.translation.z = processed_position[2]

    t.transform.rotation.x = processed_orientation[0]
    t.transform.rotation.y = processed_orientation[1]
    t.transform.rotation.z = processed_orientation[2]
    t.transform.rotation.w = processed_orientation[3]

    t.header.frame_id = _parent_frame
    t.child_frame_id = _child_frame
    t.header.stamp = rospy.Time(0)

    return t


def generateLaunchFile(_measurements, _file_name="output.launch"):

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('scene_calibration')
    output_file_path = os.path.join(package_path, 'output', _file_name)
    with open(output_file_path, 'w') as launch_file:
        launch_file.write(f"<?xml version=\"1.0\"?>\n")
        launch_file.write(f"<launch>\n")
        for measurement in _measurements:
            launch_file.write(
                f"  <node pkg=\"tf\" type=\"static_transform_publisher\" name=\"measurement_tf_{rospy.Time.now()}\" args=\"{measurement.transform.translation.x} {measurement.transform.translation.y} {measurement.transform.translation.z} {measurement.transform.rotation.x} {measurement.transform.rotation.y} {measurement.transform.rotation.z} {measurement.transform.rotation.w} {measurement.header.frame_id} {measurement.child_frame_id } 100\" />\n")
        launch_file.write(f"</launch>\n")
    rospy.loginfo(f"Launch exported to {package_path}/output/"+_file_name)



def renameChildFrame(_measurements,_target_frames):

    if len(_measurements) != len(_target_frames):
        print("Measurements and TargetFrames array have different size. Quiting...")
        sys.exit()

    for i in range(len(_measurements)):
        _measurements[i].child_frame_id = _target_frames[i]

def process_transforms(_transforms):

    # Verificando se todos os TransformStamped têm o mesmo parent_frame
    parent_frame = _transforms[0].header.frame_id
    for i in range(0,len(_transforms)):
        if _transforms[i].header.frame_id != parent_frame:
            print("Error: Not all TransformStamped have the same parent_frame. Exiting the program.")
            sys.exit()

    # Se todos os TransformStamped têm o mesmo parent_frame, aplicar a transformação homogênea
    for i in range(1, len(_transforms)):
        _transforms[i] = apply_inverse_transform(_transforms[0], _transforms[i])
        _transforms[i].header.frame_id = _transforms[0].child_frame_id

    #return  transforms

def apply_inverse_transform(_transform1, _transform2):

    # Convertendo TransformStamped para matrizes 4x4
    trans1 = [_transform1.transform.translation.x, _transform1.transform.translation.y, _transform1.transform.translation.z]
    rot1 = [_transform1.transform.rotation.x, _transform1.transform.rotation.y, _transform1.transform.rotation.z, _transform1.transform.rotation.w]
    mat1 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans1), tf.transformations.quaternion_matrix(rot1))

    trans2 = [_transform2.transform.translation.x, _transform2.transform.translation.y, _transform2.transform.translation.z]
    rot2 = [_transform2.transform.rotation.x, _transform2.transform.rotation.y, _transform2.transform.rotation.z, _transform2.transform.rotation.w]
    mat2 = tf.transformations.concatenate_matrices(tf.transformations.translation_matrix(trans2), tf.transformations.quaternion_matrix(rot2))

    # Aplicando a transformação homogênea inversa do primeiro TransformStamped
    mat1_inv = np.linalg.inv(mat1)
    mat3 = np.dot(mat1_inv, mat2)

    # Convertendo a matriz resultante de volta para um TransformStamped
    transform3 = TransformStamped()
    transform3.header.frame_id = _transform1.child_frame_id
    transform3.child_frame_id = _transform2.child_frame_id
    transform3.transform.translation.x, transform3.transform.translation.y, transform3.transform.translation.z = [round(val, 6) for val in tf.transformations.translation_from_matrix(mat3)]
    transform3.transform.rotation.x, transform3.transform.rotation.y, transform3.transform.rotation.z, transform3.transform.rotation.w = [round(val, 6) for val in tf.transformations.quaternion_from_matrix(mat3)]

    return transform3

def main():

    rospy.init_node('tf_measurement_node')
    operation_mode = rospy.get_param('~operation_mode', 0)
    tf_listener = tf.TransformListener()
    parent_frame = rospy.get_param('~parent_frame', 'base_link')
    child_frame = rospy.get_param('~child_frame', 'child_frame')
    target_frames = rospy.get_param('~target_frames', ['target_frame0'])
    planes = rospy.get_param('~planes', ['0_plane', '1_plane'])
    timeout = rospy.get_param('~timeout', 10)
    num_samples = rospy.get_param('~num_samples', 10)
    delay_between_samples = rospy.get_param('~delay_between_samples', 0.5)
    plane_z_offset = rospy.get_param('~plane_z_offset', 0.1)

    measurements = []
    measurement = TransformStamped()

    if operation_mode == Method.SINGLE_FRAME.value:
        if len(target_frames) > 1:
            rospy.logerr("The frame calibration supports only target frames with size = 1")
        else:
            rospy.loginfo(f">>>>>Place the reference to create {target_frames} . Press Enter to start the measurement.")
            input()
            measurement = getMeasurement(tf_listener, parent_frame, child_frame, num_samples, delay_between_samples, timeout)
            measurements.append(measurement)
            renameChildFrame(measurements, target_frames)
            generateLaunchFile(measurements)

    elif operation_mode == Method.MULTI_FRAME.value:
        if len(target_frames) < 2:
            rospy.logerr("Target frames array should be size larger than one.")
        else:
            for target_frame in target_frames:
                rospy.loginfo(f">>>>>Place the reference to create {target_frame} . Press Enter to start the measurement.")
                input()
                measurement = getMeasurement(tf_listener, parent_frame, child_frame, num_samples, delay_between_samples, timeout)
                measurements.append(measurement)
                rospy.loginfo(f" The {target_frame} has been created.")
            renameChildFrame(measurements, target_frames)
            process_transforms(measurements)
            generateLaunchFile(measurements)

#   TODO: the plane_z_offset does not work in the method
    elif operation_mode == Method.MULTI_PLANES_PRECISE.value:
        if len(target_frames) > 1:
            rospy.logerr("The multi plane calibration supports only target frames with size = 1")
        else:
            rospy.loginfo(f">>>>>Put the reference on origin. Press Enter to start the measurement.")
            input()
            measurements.append(getMeasurement(tf_listener, parent_frame, child_frame, num_samples, delay_between_samples, timeout))
            renameChildFrame(measurements, target_frames)
            points = ['p0', 'px', 'py']
            plane_measurements = []
            plane_measurements.append(measurements[0])
            for plane in planes:
                rospy.loginfo(f">>>>>Defining the {plane}...")
                for point in points:
                    rospy.loginfo(f"Put the reference on {point}. Press Enter to start the measurement.")
                    input()
                    plane_measurement = getMeasurement(tf_listener, parent_frame, child_frame, num_samples, delay_between_samples, timeout)
                    plane_measurement.child_frame_id = plane + '_' + point
                    plane_measurements.append(plane_measurement)
                    rospy.loginfo(f"The {point} has been created.")
                rospy.loginfo(f"The {plane} has been created.")

            process_transforms(plane_measurements)
            origin = plane_measurements.pop(0)
            generateYamlDescription(origin, plane_measurements)
            generateLaunchFile([origin])
            generateLaunchFile(plane_measurements, "planes.launch")

    elif operation_mode == Method.MULTI_PLANES.value:
        if len(target_frames) > 1:
            rospy.logerr("The multi plane calibration supports only target frames with size = 1")

        rospy.loginfo(f">>>>>Put the reference on origin. Press Enter to start the measurement.")
        input()
        measurements.append(getMeasurement(tf_listener, parent_frame, child_frame, num_samples, delay_between_samples, timeout))
        renameChildFrame(measurements, target_frames)

        for plane in planes:
            rospy.loginfo(f">>>>>Defining the {plane}...")
            rospy.loginfo(f"Put the reference. Press Enter to start the measurement.")
            input()
            plane_measurement = getMeasurement(tf_listener, parent_frame, child_frame, num_samples, delay_between_samples, timeout)
            plane_measurement.child_frame_id = plane
            measurements.append(plane_measurement)
            rospy.loginfo(f"The {plane} has been created.")

        process_transforms(measurements)
        origin = measurements.pop(0)
        generateYamlDescription(origin, generate_planes_from_frames(measurements,plane_z_offset))
        generateLaunchFile([origin])
        generateLaunchFile(measurements, "planes.launch")

    else:
        rospy.logerr("Operation mode not supported.")


if __name__ == '__main__':
    main()
