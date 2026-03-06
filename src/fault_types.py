#!/usr/bin/env python3
#
# Copyright [2026] [Min Hee Jo MinHee.Jo@de.bosch.com]

# Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with the License. You may obtain a copy of the License at

#    http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

from typing import Union
import random


def random_value(range: tuple) -> Union[int, float]:
    """
    Return random values according to the data type.

    Keyword arguments:
    range -- range with minimum and maximum value
    """
    if isinstance(range[0], float) or isinstance(range[1], float):
        return random.uniform(range[0], range[1])
    elif isinstance(range[0], int) and isinstance(range[1], int):
        return random.randint(range[0], range[1])


def add(base: Union[int, float],
        offset: Union[int, float]) -> Union[int, float]:
    """
    Return added value of base and offset.

    Keyword arguments:
    base -- base value
    offset -- offset to add
    """
    return base + offset


def random_add(base: Union[int, float], range: tuple) -> Union[int, float]:
    """
    Return added value of base and random offset.
    The random value is determined according to the data types of the
    input range values.

    Keyword arguments:
    base -- base value
    range -- range with minimum and maximum value
    """
    random_offset = random_value(range)

    return add(base, random_offset)


def is_empty_operation(fault_type: str, fault_values: tuple) -> bool:
    """
    Check an empty operation.

    Keyword arguments:
    fault_type -- fault type
    fault_values -- fault values
    """
    # fault types
    if fault_type == "add" and fault_values[0] == 0:
        return True
    elif (fault_type == "random_add" and
          fault_values[0] == 0 and
          fault_values[1] == 0):
        return True
    else:
        return False


def operation(fault_type: str,
              base: Union[int, float],
              fault_values: tuple) -> Union[int, float]:
    """
    Return the operation results according to the fault type.

    Keyword arguments:
    fault_type -- fault type
    base -- base value
    fault_values -- fault values
    """
    # fault types
    if fault_type == "add":
        result = add(base, fault_values[0])
    elif fault_type == "replace":
        result = fault_values[0]
    elif fault_type == "random_add":
        result = random_add(base, fault_values)
    elif fault_type == "random_replace":
        result = random_value(fault_values)
    else:
        raise ValueError("Undefined fault_type")

    return result