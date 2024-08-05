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

from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore
from pathlib import Path
from datetime import datetime
from functools import reduce
from typing import Tuple
import plotly.graph_objs as go
import pandas as pd
import read_yaml
import random


class Rosbag_Plot:
    """
    Create and template a pandas dataframe for original and fault
    injected rosbags.
    """
    def __init__(self):
        """
        Initialize Rosbag_Plot class and create figure object.
        """
        # figure object
        self.fig = go.Figure()


    def add_scatter_plot(self,
                         x: pd.DataFrame,
                         y: pd.DataFrame,
                         name: str,
                         marker_symbol: str,
                         marker_color: str,
                         marker_size: float = 3.5,
                         maker_opacity: float = 0.8):
        """
        Add a scatter plot trace.

        Keyword arguments:
        x -- x axis data frame
        y -- y axis data frame
        name -- trace name
        marker_symbol -- marker symbol
        marker_color -- marker color
        marker_size -- marker size [default: 3.5]
        maker_opacity -- marker opacity [default: 0.8]
        """
        self.fig.add_trace(
            go.Scatter(
                x=x, y=y,
                name=name,
                mode='markers',
                marker=dict(
                    symbol=marker_symbol,
                    color=marker_color,
                    size=marker_size,
                    opacity=maker_opacity
                )
            )
        )


    def update_layout(self,
                      title: str,
                      xaxis_title: str,
                      yaxis_title: str,
                      title_font_size: int = 25):
        """
        Update the layout of the plots.

        Keyword arguments:
        title -- plot title
        title_font_size -- title font size [default: 25]
        xaxis_title -- x axis title
        yaxis_title -- y axis title
        """
        self.fig.update_layout(
            plot_bgcolor='white',
            title_text=title,
            title_font_size=title_font_size,
            xaxis=dict(
                title=xaxis_title,
                tickangle=45,
                tickformat='%10d'
            ),
            yaxis=dict(
                title=yaxis_title
            )
        )
        self.fig.update_xaxes(
            mirror=True,
            ticks='outside',
            showline=True,
            linecolor='black',
            gridcolor='lightgrey'
        )
        self.fig.update_yaxes(
            mirror=True,
            ticks='outside',
            showline=True,
            linecolor='black',
            gridcolor='lightgrey'
        )


    def write_html(self, path: str):
        """
        Write the plots in html file.

        Keyword arguments:
        path -- path where to save html file
        """
        self.fig.write_html(path)


class Dataframe_For_Plot:
    """
    Extract dataframes from rosbags.
    """
    def __init__(self, config_path: str):
        """
        Initialize Dataframe_For_Plot class and its variables.

        Keyword arguments:
        config_path -- relative path to configuration YAML file
        """
        self.cur_dir = Path.cwd()

        self.yaml = read_yaml.Yaml(self.cur_dir, config_path)

        # read config.yaml
        self.yaml.read_yaml()

        # read parameters
        self.parameters = self.yaml.get_params()

        # plot data
        plot_df_columns = [
                'rosbag',
                'topic',
                'type',
                'timestamp',
                'value'
            ]
        self.data = [plot_df_columns]

        # filter data
        filter_df_columns = [
                'rosbag',
                'topic',
                'type',
                'timestamp',
                'value',
                'window_start_timestamp',
                'window_end_timestamp'
            ]
        self.filtered_df = pd.DataFrame(columns=filter_df_columns)


    def get_df(self) -> pd.DataFrame:
        """
        Return a dataframe from the original rosbags and fault injected
        rosbags.
        """
        # input from the first parameter set
        first_input = self.yaml.get_input(self.parameters[0]) 
        # subdirectories from the input from the first parameter set
        input_as_parent_dir = self.yaml.get_subdirectory(first_input)

        # all rosbags in directory
        if input_as_parent_dir:
            for rosbag in input_as_parent_dir:
                # read each parameters for the rosbag
                self.yaml.set_param(self.parameters[0])

                # input rosbag
                input = self.cur_dir / rosbag
                # read every message
                self.read_rosbag(input, "before")

                # output rosbag
                output = Path(str(input).replace("input", "output"))
                # read every message
                self.read_rosbag(output, "after")
        # specific rosbag
        else:
            for parameter in self.parameters:
                # read each parameters for the rosbag
                self.yaml.set_param(parameter)

                # input rosbag
                input = self.cur_dir / self.yaml.get_input(parameter)
                # read every message
                self.read_rosbag(input, "before")

                # output rosbag
                output = Path(str(input).replace("input", "output"))
                # read every message
                self.read_rosbag(output, "after")

        return pd.DataFrame(self.data[1:], columns=self.data[0])


    def read_rosbag(self, rosbag: Path, suffix: str):
        """
        Read the original rosbag and the fault injected rosbag.

        Keyword arguments:
        rosbag -- path to the input rosbag
        suffix -- suffix to differentiate the original and fault
        injected rosbags
        """
        # initialize timestamp offset
        timestamp_offset = -1

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

        # read rosbag
        with Reader(rosbag) as reader:
            for topic in self.yaml.topics:
                # extract parameters
                name, type_, _, _, _, _ = self.yaml.get_topic_params(topic)

                # skip topics not in topic mask filter
                if name not in self.yaml.topic_mask_filter:
                    continue

                # message type
                msg_type = 'msg' + type_

                for connection, timestamp, rawdata in reader.messages():
                    if connection.topic == name:
                        # deserialize the message
                        msg = typestore.deserialize_cdr(
                            rawdata, connection.msgtype
                            )

                        if timestamp_offset == -1:
                            timestamp_offset = timestamp

                        # data -> ['rosbag', 'topic', 'type', 'timestamp', 'value']
                        self.data.append([
                            rosbag.name + "_" + suffix,
                            name,
                            type_,
                            (timestamp - timestamp_offset) / 1e9,
                            eval(msg_type)
                            ])


    def filter_df(self, df: pd.DataFrame) -> Tuple[pd.DataFrame, pd.DataFrame]:
        """
        Return the filtered dataframe and the dataframe for what has
        been filtered according to the parameters configured in YAML
        file.

        Keyword arguments:
        df -- pandas dataframe
        """
        # input from the first parameter set
        first_input = self.yaml.get_input(self.parameters[0]) 
        # subdirectories from the input from the first parameter set
        input_as_parent_dir = self.yaml.get_subdirectory(first_input)

        # all rosbags in directory
        if input_as_parent_dir:
            for rosbag in input_as_parent_dir:
                # read each parameters for the rosbag
                self.yaml.set_param(self.parameters[0])

                # filter
                if self.yaml.filters:
                    df = self.filter_rosbag(df, rosbag.name + "_after")
        # specific rosbag
        else:
            for parameter in self.parameters:
                # read each parameters for the rosbag
                self.yaml.set_param(parameter)

                # rosbag
                rosbag = self.yaml.get_input(parameter).rsplit('/', 1)

                # filter
                if self.yaml.filters:
                    df = self.filter_rosbag(df, rosbag[-1] + "_after")

        return df, self.filtered_df


    def filter_rosbag(self, df: pd.DataFrame, rosbag: str) -> pd.DataFrame:
        """
        Filter the dataframe according to the parameters configured in
        YAML file.

        Keyword arguments:
        df -- pandas dataframe
        rosbag -- rosbag name
        """
        for filter in self.yaml.filters:
            # extract filter parameters
            topic, type_, filter_type, filter_values, filter_window_size, pass_size = self.yaml.get_filter_params(filter)

            conditions = [
                df['rosbag'] == rosbag,
                df['topic'] == topic,
                df['type'] == type_
            ]

            # filter types
            if filter_type == "show_amplitude_greater_than_or_equal_to":
                # drop values smaller than or equal as filter_values
                conditions.append(df['value'] < filter_values[0])
            elif filter_type == "show_amplitude_less_than_or_equal_to":
                # drop values bigger than or equal as filter_values
                conditions.append(df['value'] > filter_values[0])
            elif filter_type == "show_amplitude_between":
                # drop values out of range of filter_values
                conditions.append(
                    (df['value'] < filter_values[0]) |
                    (filter_values[1] < df['value'])
                    )
            else:
                raise ValueError("Undefined filter_type")

            combined_condition = reduce(lambda x, y: x & y, conditions)

            # window size given
            if filter_window_size > 0:
                # filters on the sliding window
                for group in df.rolling(window = filter_window_size):
                    # did not meet the pass size
                    if len(group.loc[~combined_condition]) < pass_size:
                        unsatisfied = group.loc[combined_condition].copy()
                        
                        unsatisfied['window_start_timestamp'] = group.iloc[0,:]['timestamp']
                        unsatisfied['window_end_timestamp'] = group.iloc[-1,:]['timestamp']

                        self.filtered_df = pd.concat([
                            self.filtered_df,
                            unsatisfied
                            ])

            # filter globally
            df = df[~combined_condition]

        return df


if __name__ == '__main__':
    import sys

    config_path = sys.argv[1]

    dataframe_plot = Dataframe_For_Plot(config_path)
    df = dataframe_plot.get_df()
    df, filtered_df = dataframe_plot.filter_df(df)

    # rosbag list
    rosbags = df['rosbag'].unique()

    # for all rosbags
    plot_all_result = Rosbag_Plot()

    for rosbag in rosbags:
        # filter by rosbag
        rosbag_filtered_df = df[df['rosbag'] == rosbag]

        # topic list
        topics = rosbag_filtered_df['topic'].unique()

        for topic in topics:
            # filter again by topic
            topic_filtered_df = rosbag_filtered_df[
                (rosbag_filtered_df['topic'] == topic)
                ]

            # type list
            types = topic_filtered_df['type'].unique()

            for type_ in types:
                # filter again by type
                type_filtered_df = topic_filtered_df[
                    (topic_filtered_df['type'] == type_)
                    ]

                # different color for before and after
                if rosbag.endswith("before"):
                    r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
                    symbol = 'diamond'
                else:
                    r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)
                    symbol = 'circle'

                # recolor when low contrast
                while any(60 < value < 190 for value in (r, g, b)):
                    r, g, b = random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)

                color = f"rgb({r},{g},{b})"
                
                plot_all_result.add_scatter_plot(
                    x=type_filtered_df['timestamp'],
                    y=type_filtered_df['value'],
                    name=rosbag + "<br>" + topic + "<br>" + type_,
                    marker_symbol=symbol,
                    marker_color=color
                    )

    # all rosbags
    plot_all_result.update_layout(
        title="Comparison Between Original and Fault-Injected Topics for all rosbags",
        xaxis_title="Timestamp (sec)",
        yaxis_title="Values"
        )

    # write plot as html
    date_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    plot_all_file_name = "./plot/plot_" + date_time + ".html"
    plot_all_result.write_html(plot_all_file_name)
    print("Plot file written at " + plot_all_file_name)

    csv_file_name = "./csv/filtered_" + date_time + ".csv"
    filtered_df.to_csv(csv_file_name, index=False)
    
    del plot_all_result
    del dataframe_plot