#!/usr/bin/env python3
from rosbags.rosbag1 import Reader, Writer
from rosbags.typesys import get_types_from_msg, get_typestore, Stores, base
from typing import Callable, Dict


def read_ros1_do_stuff_save_ros1(input_path: str, output_path: str, do_stuff: Dict[str, Callable] = {}):
    """
    Reads a ROS1 bag file, processes messages for specified topics using provided callables,
    and saves the result in a new bag with the same message types and topic names.
    This function is actually INDEPENDENT of ROS

    Args:
        input_path (str): Path to the input ROS1 bag.
        output_path (str): Path to the output ROS1 bag.
        do_stuff (Dict[str, Callable]): A dictionary where keys are topic names and values are callables
                                        that process messages. Each callable takes a message and returns a message.
    """
    typestore = get_typestore(Stores.ROS1_NOETIC)
    # Open the input bag for reading
    with Reader(input_path) as reader:
        # Open the output bag for writing
        with Writer(output_path) as writer:
            # Map input connections to output connections
            conn_map = {}

            # Add connections to the output bag
            for connection in reader.connections:
                try:
                    conn = writer.add_connection(connection.topic, connection.msgtype, typestore=typestore)
                    conn_map[connection.id] = conn
                except base.TypesysError as e:
                    print(f"Warning: {e}")

            # Iterate through messages in the input bag
            for connection, timestamp, rawdata in reader.messages():
                if connection.id not in conn_map:
                    continue
                # Deserialize the message
                msg = typestore.deserialize_ros1(rawdata, connection.msgtype)

                # Check if there's a callable for this topic
                if connection.topic in do_stuff:
                    # Process the message
                    msg = do_stuff[connection.topic](msg)

                # Get the corresponding output connection
                out_conn = conn_map[connection.id]

                # Serialize the message
                rawdata_out = typestore.serialize_ros1(msg, connection.msgtype)

                # Write the message to the output bag
                writer.write(out_conn, timestamp, rawdata_out)
