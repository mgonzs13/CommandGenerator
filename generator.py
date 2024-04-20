
import re
import json
import rclpy
import warnings
from tqdm import tqdm
from gpsr_demo import GpsrDemo
from gpsr_commands import CommandGenerator


def read_data(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    return data


def parse_names(data):
    parsed_names = re.findall(r'\|\s*([A-Za-z]+)\s*\|', data, re.DOTALL)
    parsed_names = [name.strip() for name in parsed_names]

    if parsed_names:
        return parsed_names[1:]
    else:
        warnings.warn(
            "List of names is empty. Check content of names markdown file")
        return []


def parse_locations(data):
    parsed_locations = re.findall(
        r'\|\s*([0-9]+)\s*\|\s*([A-Za-z,\s, \(,\)]+)\|', data, re.DOTALL)
    parsed_locations = [b for (a, b) in parsed_locations]
    parsed_locations = [location.strip() for location in parsed_locations]

    parsed_placement_locations = [
        location for location in parsed_locations if location.endswith('(p)')]
    parsed_locations = [location.replace('(p)', '')
                        for location in parsed_locations]
    parsed_placement_locations = [location.replace(
        '(p)', '') for location in parsed_placement_locations]
    parsed_placement_locations = [location.strip()
                                  for location in parsed_placement_locations]
    parsed_locations = [location.strip() for location in parsed_locations]

    if parsed_locations:
        return parsed_locations, parsed_placement_locations
    else:
        warnings.warn(
            "List of locations is empty. Check content of location markdown file")
        return []


def parse_rooms(data):
    parsed_rooms = re.findall(r'\|\s*(\w+ \w*)\s*\|', data, re.DOTALL)
    parsed_rooms = [rooms.strip() for rooms in parsed_rooms]

    if parsed_rooms:
        return parsed_rooms[1:]
    else:
        warnings.warn(
            "List of rooms is empty. Check content of room markdown file")
        return []


def parse_objects(data):
    parsed_objects = re.findall(r'\|\s*(\w+)\s*\|', data, re.DOTALL)
    parsed_objects = [
        objects for objects in parsed_objects if objects != 'Objectname']
    parsed_objects = [objects.replace("_", " ") for objects in parsed_objects]
    parsed_objects = [objects.strip() for objects in parsed_objects]

    parsed_categories = re.findall(
        r'# Class \s*([\w,\s, \(,\)]+)\s*', data, re.DOTALL)
    parsed_categories = [category.strip() for category in parsed_categories]
    parsed_categories = [category.replace('(', '').replace(
        ')', '').split() for category in parsed_categories]
    parsed_categories_plural = [category[0] for category in parsed_categories]
    parsed_categories_plural = [category.replace(
        "_", " ") for category in parsed_categories_plural]
    parsed_categories_singular = [category[1]
                                  for category in parsed_categories]
    parsed_categories_singular = [category.replace(
        "_", " ") for category in parsed_categories_singular]

    if parsed_objects or parsed_categories:
        return parsed_objects, parsed_categories_plural, parsed_categories_singular
    else:
        warnings.warn(
            "List of objects or object categories is empty. Check content of object markdown file")
        return []


if __name__ == "__main__":
    names_file_path = 'files/names/names.md'
    locations_file_path = 'files/maps/location_names.md'
    rooms_file_path = 'files/maps/room_names.md'
    objects_file_path = 'files/objects/test.md'

    names_data = read_data(names_file_path)
    names = parse_names(names_data)

    locations_data = read_data(locations_file_path)
    location_names, placement_location_names = parse_locations(locations_data)

    rooms_data = read_data(rooms_file_path)
    room_names = parse_rooms(rooms_data)

    objects_data = read_data(objects_file_path)
    object_names, object_categories_plural, object_categories_singular = parse_objects(
        objects_data)

    generator = CommandGenerator(
        names,
        location_names,
        placement_location_names,
        room_names,
        object_names,
        object_categories_plural,
        object_categories_singular
    )

    gpsr_data = []

    rclpy.init()
    gpsr_demo = GpsrDemo()

    try:
        for _ in tqdm(range(10)):
            command = generator.generate_command_start(cmd_category="")
            response = gpsr_demo.send_prompt(command)
            gpsr_data.append({"command": command, "response": response})

    except KeyboardInterrupt:
        gpsr_demo.cancel()
    finally:
        with open("sample.json", 'w') as output_file:
            json.dump(gpsr_data, output_file, indent=4)
        rclpy.shutdown()
