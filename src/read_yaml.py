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

from pathlib import Path
from typing import Tuple, Union
import yaml


class Yaml():
    """
    Load, read, format, set the parameters configured in YAML file.
    """
    def __init__(self, cur_dir, config_path):
        """
        Initialize Yaml class and its variables.

        Keyword arguments:
        cur_dir -- current working directory
        config_path -- relative path to configuration YAML file
        """
        try:
            self.cur_dir = cur_dir.resolve(strict=True)
        except FileNotFoundError:
            print("Current directory not valid")

        if not isinstance(config_path, str):
            raise ValueError("Configuration file path is not a string type: {}".format(config_path))
        self.config_path = config_path

        self.topic_mask_filter = []
        self.topics = []
        self.filters = []


    @property
    def DEFAULT_SEED(self):
        """
        Return the default value of the seed.
        """
        return 1


    def read_yaml(self):
        """
        Read and load the configuration YAML file.
        """
        with open (self.cur_dir / self.config_path, "r") as f:
            try:
                self.config = yaml.safe_load(f)
            except yaml.YAMLError as exc:
                if hasattr(exc, 'problem_mark'):
                    mark = exc.problem_mark
                    print ("Could not load YAML file due to error (See position: line {} column {})".format(mark.line + 1, mark.column + 1))


    def get_seed(self) -> int:
        """
        Return the seed parameter.
        Return the default seed if no seed is configured.
        """
        if 'seed' in self.config:
            seed = self.config['seed']
        else:
            seed = self.DEFAULT_SEED()
        
        if not isinstance(seed, int):
            raise ValueError("seed parameter is not an integer type: {}".format(seed))

        return seed
    

    def get_custom_message_path(self) -> list:
        """
        Return a list of relative paths to custom message files.
        Return a empty list if no custom message path is configured.
        """
        custom_message_path = []

        # optional custom message path parameter
        if 'custom_message_path' in self.config:
            custom_message_path = self.config['custom_message_path']

            if not isinstance(custom_message_path, list):
                raise ValueError("custom_message_path parameter is not a list type: {}".format(custom_message_path))

            # delete the first character if it starts with / 
            return [s[1:] if s.startswith('/') else s for s in custom_message_path]
        else:
            return []


    def get_params(self) -> list:
        """
        Return a list of parameters.
        """
        return self.config['parameters']
    

    def get_input(self, parameter: list) -> str:
        """
        Return the input parameter

        Keyword arguments:
        parameter -- list of one parameter set
        """
        return parameter['input']


    def set_param(self, parameter: list):
        """
        Set the subparameters of the input parameter list.
        Post-process the format of each subparameter.
        
        Keyword arguments:
        parameter -- list of one parameter set
        """
        # topics masking list
        self.topic_mask_filter = [x if x.startswith('/') else '/' + x for x in parameter['topic_mask_filter']]
    
        if not isinstance(self.topic_mask_filter, list):
            raise ValueError("topic_mask_filter parameter is not a list type: {}".format(self.topic_mask_filter))

        # list of topics
        self.topics = parameter['topics']

        if not isinstance(self.topics, list):
            raise ValueError("topics parameter is not a list type: {}".format(self.topics))

        for topic in self.topics:
            if not isinstance(topic, dict):
                raise ValueError("topic parameter is not a dict type: {}".format(topic))

            topic['topic'] = topic['topic'] if topic['topic'].startswith('/') else '/' + topic['topic']
            topic['type'] = topic['type'] if topic['type'].startswith('.') else '.' + topic['type']

        # optional filter parameter
        if 'filters' in parameter:
            # list of filters
            self.filters = parameter['filters']
        
            if not isinstance(self.filters, list):
                raise ValueError("filters parameter is not a list type: {}".format(self.filters))

            for filter in self.filters:
                if not isinstance(filter, dict):
                    raise ValueError("filter parameter is not a dict type: {}".format(filter))

                filter['topic'] = filter['topic'] if filter['topic'].startswith('/') else '/' + filter['topic']
                filter['type'] = filter['type'] if filter['type'].startswith('.') else '.' + filter['type']
        else:
            self.filters = []


    def get_topic_params(self, topic: dict) -> Tuple[str, str, str, Tuple[Union[int, float], Union[int, float]], Union[int, float], Union[int, float]]:
        """
        Return subparameters of the input topic dictionary.
        
        Keyword arguments:
        topic -- dictionary of one topic parameter set
        """
        # topic name
        name = topic['topic']

        if not isinstance(name, str):
            raise ValueError("topic parameter is not a string type: {}".format(name))

        # message type
        msg_type = topic['type']

        if not isinstance(msg_type, str):
            raise ValueError("type parameter is not a string type: {}".format(msg_type))

        # fault type
        fault_type = topic['fault_type']

        if not isinstance(fault_type, str):
            raise ValueError("fault_type parameter is not a string type: {}".format(fault_type))

        # fault values
        fault_value = topic['fault_value']

        # post process fault values
        if fault_type.startswith('random_'):
            if not isinstance(fault_value, str):
                raise ValueError("fault_value parameter is not a range: {}".format(fault_value))

            fault_value = [self.string_to_num(x.strip()) for x in fault_value.split(',')]

            if fault_value[0] > fault_value[1]:
                raise ValueError("Minimum value of fault_value is bigger than its maximum value in topic {}".format(name))

            fault_values = (fault_value[0], fault_value[1])
        else:
            if not isinstance(fault_value, (int, float)):
                raise ValueError("fault_value parameter is not an integer or a float type: {}".format(fault_value))
            
            fault_values = (self.string_to_num(fault_value), None)
        
        # start fault injection after time in seconds 
        if 'start_after_sec' in topic:
            start_after_sec = topic['start_after_sec']

            if not isinstance(start_after_sec, (int, float)):
                raise ValueError("start_after_sec parameter is not an integer or a float type: {}".format(start_after_sec))
            elif start_after_sec < 0:
                raise ValueError("start_after_sec parameter is a negative value: {}".format(start_after_sec))
        else:
            start_after_sec = 0

        # fault injection duration in seconds 
        if 'duration_sec' in topic:
            duration_sec = topic['duration_sec']

            if not isinstance(duration_sec, (int, float)):
                raise ValueError("duration_sec parameter is not an integer or a float type: {}".format(duration_sec))
            elif duration_sec < 0:
                raise ValueError("duration_sec parameter is a negative value: {}".format(duration_sec))
        else:
            duration_sec = 0

        return name, msg_type, fault_type, fault_values, start_after_sec, duration_sec
    

    def get_filter_params(self, filter: list) -> Tuple[str, str, str, Tuple[Union[int, float], Union[int, float]], int, int]:
        """
        Return subparameters of the input filter list.
                
        Keyword arguments:
        filter -- list of filter parameter set
        """
        # topic name
        name = filter['topic']

        if not isinstance(name, str):
            raise ValueError("topic parameter is not a string type: {}".format(name))

        # message type
        msg_type = filter['type']

        if not isinstance(msg_type, str):
            raise ValueError("type parameter is not a string type: {}".format(msg_type))
        
        # filter type
        filter_type = filter['filter_type']

        if not isinstance(filter_type, str):
            raise ValueError("filter_type parameter is not a string type: {}".format(filter_type))
        
        # filter values
        filter_value = filter['filter_value']
        
        # post process filter values
        if filter_type.endswith('_between'):
            if not isinstance(filter_value, str):
                raise ValueError("filter_value parameter is not a range: {}".format(filter_value))

            filter_value = [self.string_to_num(x.strip()) for x in filter_value.split(',')]

            if filter_value[0] > filter_value[1]:
                raise ValueError("Minimum value of filter_value is bigger than its maximum value in topic {}".format(name))

            filter_values = (filter_value[0], filter_value[1])
        else:
            if not isinstance(filter_value, (int, float)):
                raise ValueError("filter_value parameter is not an integer or a float type: {}".format(filter_value))
            
            filter_values = (self.string_to_num(filter_value), None)

        # window size for filtering
        filter_window_size = filter['filter_window_size']

        if not isinstance(filter_window_size, int):
            raise ValueError("filter_window_size parameter is not an integer type: {}".format(filter_window_size))
        elif filter_window_size < 0:
            raise ValueError("filter_window_size parameter is a negative value: {}".format(filter_window_size))

        # pass criteria within window size
        if 0 < filter_window_size:
            pass_size = filter['pass_size']
        else:
            pass_size = 0

        if not isinstance(pass_size, int):
            raise ValueError("pass_size parameter is not an integer type: {}".format(pass_size))
        elif pass_size < 0:
            raise ValueError("pass_size parameter is a negative value: {}".format(pass_size))
        elif 0 < filter_window_size < pass_size:
            raise ValueError("pass_size parameter is bigger than the filter_window_size: {} > {}".format(pass_size, filter_window_size))

        return name, msg_type, filter_type, filter_values, filter_window_size, pass_size
    

    def string_to_num(self, s):
        """
        Return the string as integer or float.
                        
        Keyword arguments:
        s -- string to convert
        """
        if isinstance(s, (int, float)):
            return s
        elif s.isdigit():
            return int(s)
        else:
            return float(s)


    def get_subdirectory(self, path: str) -> list:
        """
        Return a list of subdirectories of rosbags under the input path.
                        
        Keyword arguments:
        path -- parent path
        """
        path = Path(path)
        
        subdirectories = list(path.glob('./*/*.db3'))
        subdirectories = [x.parent for x in subdirectories]

        return subdirectories


    def no_topic(self):
        """
        Check if there is no topics to inject faults.
        """
        if not self.topic_mask_filter:
            raise Exception("No faults to inject")
    

    def undefined_topics(self):
        """
        Check if there are undefined topics to inject faults.
        """
        # get only topic names from topics
        topic_names = self.get_values_list_of_dict(self.topics, "topic")

        # check undefined topics in topic mask filter
        undefined = [x for x in self.topic_mask_filter if x not in set(topic_names)]

        if undefined:
            raise Exception("Undefined topics is in topic_mask_filter")
    

    def get_values_list_of_dict(self, list_of_dicts: list, key) -> list:
        """
        Return a list of all values in a dictionary that matches with
        the input key.
                        
        Keyword arguments:
        list_of_dicts -- list of dictionaries
        key -- key to search in a dictionary
        """
        nth_values = []
        for dict in list_of_dicts:
            nth_values.append(dict[key])

        return nth_values
        

    def mismatching_datatype(self, s: str, fault_values: tuple):
        """
        Check if fault value types are mismaching the message definition
        data types.
                                
        Keyword arguments:
        s -- base string
        fault_values -- fault values
        """
        string_in_num = self.string_to_num(s)

        # fault_value given as range
        if all(fault_values):
            if type(fault_values[0]) == type(fault_values[1]) == type(string_in_num):
                pass
            else:
                raise ValueError("The data type of fault_value parameter {} should be: {}".format(fault_values, type(string_in_num)))
        # fault_value given as value
        else:
            if type(fault_values[0]) == type(string_in_num):
                pass
            else:
                raise ValueError("The data type of fault_value parameter {} should be: {}".format(fault_values, type(string_in_num)))
            

    def guess_msgtype(self, path: Path) -> str:
        """
        Guess the message type from a message file path.

        Keyword arguments:
        path -- path to message file path
        """
        name = path.relative_to(path.parents[2]).with_suffix('')

        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name

        return str(name)