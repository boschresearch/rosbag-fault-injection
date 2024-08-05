#!/usr/bin/env python3
#
# Copyright (c) 2024 Robert Bosch GmbH and its subsidiaries.
# This program and the accompanying materials are made available under
# the terms of the Bosch Internal Open Source License v4
# which accompanies this distribution, and is available at
# http://bios.intranet.bosch.com/bioslv4.txt

__author__ = "Min Hee Jo"
__copyright__ = "Copyright 2024, Robert Bosch GmbH"
__license__ = "BIOSL"
__version__ = "4.0"
__email__ = "minhee.jo@de.bosch.com"

from rosbags.interfaces import ConnectionExtRosbag2
from rosbags.rosbag2 import Reader, Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from typing import cast
from pathlib import Path
import fault_types
import read_yaml
import random


class Fault_Injection:
    """
    Inject faults to rosbags according to the parameters configured in
    YAML file.
    """
    def __init__(self, config_path: str):
        """
        Initialize Fault_Injection class and its variables.

        Keyword arguments:
        config_path -- relative path to configuration YAML file
        """
        self.cur_dir = Path.cwd()

        self.yaml = read_yaml.Yaml(self.cur_dir, config_path)

        # read config.yaml
        self.yaml.read_yaml()

        # read seed
        seed = self.yaml.get_seed()
        # set random seed
        random.seed(seed)

        # read parameters
        self.parameters = self.yaml.get_params()


    def inject(self):
        """
        Inject faults as configured in YAML file.
        """
        # input from the first parameter set
        first_input = self.yaml.get_input(self.parameters[0])
        # subdirectories from the input from the first parameter set
        input_as_parent_dir = self.yaml.get_subdirectory(first_input)

        # inject faults to all rosbags in directory
        if input_as_parent_dir:
            for rosbag in input_as_parent_dir: 
                # read each parameters for the input
                self.yaml.set_param(self.parameters[0])

                # input rosbag
                input = self.cur_dir / rosbag
                # read write rosbag
                self.read_write_rosbag(input)
        # inject faults to specified rosbag
        else:
            for parameter in self.parameters:
                # read each parameters for the input
                self.yaml.set_param(parameter)

                # input rosbag
                input = self.cur_dir / self.yaml.get_input(parameter)
                # read write rosbag
                self.read_write_rosbag(input)


    def read_write_rosbag(self, input: Path):
        """
        Read the original rosbag and write a new rosbag with faults.

        Keyword arguments:
        input -- path to rosbag input
        """
        print("[Injection]:", input)

        # temporary directory
        temp = Path(str(input).replace("input", "temp"))

        if temp.exists():
            raise Exception("Temporary directory already exists in /data/temp")
        
        print("\t -> ", temp)

        # error handling of wrong config.yaml
        self.yaml.no_topic()
        self.yaml.undefined_topics()

        # writing rosbag
        writer_connections = {}
        # the first timestap for each type
        start_timestamps = {}

        # create type store to use if the bag has no message definitions
        typestore = get_typestore(Stores.ROS2_GALACTIC)
        add_types = {}

        for pathstr in self.yaml.get_custom_message_path():
            msgpath = self.cur_dir.joinpath(pathstr)
            msgdef = msgpath.read_text(encoding='utf-8')
            msgtype = get_types_from_msg(
                msgdef,
                self.yaml.guess_msgtype(msgpath)
                )
            add_types.update(msgtype)

        typestore.register(add_types)

        # read input rosbag and write output rosbag
        with Reader(input) as reader, Writer(temp) as writer:
            for connection in reader.connections:
                ext = cast(ConnectionExtRosbag2, connection.ext)
                
                writer_connections[connection.id] = writer.add_connection(
                    topic=connection.topic,
                    msgtype=connection.msgtype,
                    typestore=typestore,
                    serialization_format=ext.serialization_format,
                    offered_qos_profiles=ext.offered_qos_profiles,
                    )

                # initialize timestamp dict
                start_timestamps[connection.id] = -1

            for topic in self.yaml.topics:
                # extract parameters
                name, type_, fault_type, fault_values, start_after_sec, duration_sec = self.yaml.get_topic_params(topic)

                # skip topics not in topic mask filter
                if name not in self.yaml.topic_mask_filter:
                    continue
                # skip if no values are changed
                elif fault_types.is_empty_operation(fault_type, fault_values):
                    continue

                # message type
                msg_type = 'msg' + type_

                for connection, timestamp, rawdata in reader.messages():
                    # deserialize the message
                    msg = typestore.deserialize_cdr(rawdata, connection.msgtype)

                    if connection.topic == name:
                        # message timestamp in sec (recording timestamp)
                        timestamp_sec = timestamp / 1e9

                        # the first message
                        if start_timestamps[connection.id] == -1:
                            # first timestamp
                            start_timestamps[connection.id] = timestamp_sec

                            # start and stop time of fault injection
                            start = timestamp_sec + start_after_sec
                            if duration_sec != 0:
                                stop = start + duration_sec
                            else:
                                stop = None

                        # original value
                        original_value = eval(msg_type)
                        
                        # fault value data type should not contradict message type
                        self.yaml.mismatching_datatype(
                            original_value, fault_values
                            )

                        # inject only in between start and stop timestamp
                        if (start <= timestamp_sec and
                            (stop is None or timestamp_sec <= stop)):
                            # operation result
                            result = fault_types.operation(
                                fault_type,
                                original_value,
                                fault_values
                                )
                            
                            # update message type value
                            exec(msg_type + '= result')

                        # write rosbag
                        writer.write(
                            writer_connections[connection.id],
                            timestamp,
                            typestore.serialize_cdr(msg, connection.msgtype)
                            )


if __name__ == "__main__":
    import sys

    config_path = sys.argv[1]

    fault_inject = Fault_Injection(config_path)

    fault_inject.inject()

    del fault_inject