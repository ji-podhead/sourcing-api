#!/usr/-bin/env python3
import cv2
import numpy as np
import gi
import sys
import argparse
import xml.etree.ElementTree as ET
import json
import xmltodict
import pprint
import json
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis

# TODO:
# - Add error handling for camera connection issues.
# - MERGE group and category features into a unified structure but add the source (group/category) as metadata.
# TODO for main.py:
# - ADD presets for configurations, that need to include the type of the feature (int, float, enum, bool, string).
#   - this way we can call device.set_<type>_feature_value(name, value) directly without guessing the type.
# - return the device and camera object when starting the stream to allow to set features in runtime while the stream is already running to test out the effect.
# - make sure that the topic is properly published so webviz can display it.
# - Add ability to save and load settings from a JSON file.
# - Add ability to set features from command line arguments or from a config file.
# - Add ability to set features while streaming to see the effect in real-time.
# - Add ability to handle different pixel formats and convert them to a displayable format.
# - Add ability to handle different resolutions and frame rates.


# --- Configuration ---
CAMERA_NAME = "200.0.0.164"
DEVICE_GAIN_LEVELS = ['x1_0', 'x1_2', 'x1_5', 'x2_0', 'x3_0', 'x6_0']
TARGET_PIXEL_FORMAT = "Mono16"
SETTINGS_JSON_FILE = "camera_settings.json"

class GigEInteractiveTool:
    def __init__(self, verbose=False):
        self.camera = None
        self.stream = None
        self.current_gain_index = 0
        self.features={
            "strings":[],
            "booleans":[],
            "integers":[],
            "floats":[],
            "enumerations":[]
        }
        self.feature_groups={}
        self.deviceStats={}
        self.feature_categories={} # New attribute to store category-based features
        self.all_features_map={} # New attribute to store all top-level features for quick lookup
        
    def close(self):
        """Explicitly releases the camera object."""
        if self.camera:
            # By setting the reference to None, we allow the garbage collector
            # to destroy the object, which in turn calls g_object_unref()
            # on the underlying C object, releasing control.
            self.camera = None
            print("GigEInteractiveTool: Camera object reference released.")

    async def initialize(self,camera_identifier):
        """Dumps the GenICam XML from the camera and merges group/category features."""
        try:
            print(f"Connecting to camera: '{camera_identifier}'...")
            try:
                self.camera = Aravis.Camera.new(camera_identifier)
            except Exception as e:
                print(f"Error connecting to camera '{camera_identifier}': {e}")
                self.camera = None
            if self.camera is None:
                raise Exception(f"Camera '{camera_identifier}' not found.")
            print(f"Connected to {self.camera.get_model_name()} ({self.camera.get_device_id()})")
            try:
                device = self.camera.props.device
            except Exception as e:
                print(f"Error getting device properties: {e}")
                raise e
            genicam_xml = device.get_genicam_xml()
            print("Successfully retrieved GenICam XML.")
            pp = pprint.PrettyPrinter(indent=4)
            # Ensure all relevant feature types are forced into lists for consistent parsing
            xml_dict=xmltodict.parse(genicam_xml[0], force_list=('String', 'Enumeration', 'Integer', 'Float', 'Boolean', 'Command', 'Register', 'MaskedIntReg', 'EnumEntry', 'pFeature'))
            with open("genicam_xml.xml", "w") as xml_file:
                xml_file.write(genicam_xml[0])
            main=xml_dict.get("RegisterDescription")
            def get_feature_details(feature_name, feature_type, feature_xml_data, group_or_category_name, category):
                if not isinstance(feature_xml_data, dict):
                    print(f"Warning: Expected dictionary for feature_xml_data, but got {type(feature_xml_data)} for feature '{feature_name}' in '{group_or_category_name}'. Skipping.")
                    return {
                        "value": "parsingError",
                        "type": feature_type,
                        "tooltip": "Error parsing XML data",
                        "description": "Error parsing XML data",
                    }
                details = {}
                tooltip = feature_xml_data.get("ToolTip", "")
                description = feature_xml_data.get("Description", tooltip)
                min,max=None,None
                if feature_xml_data.get("pMin") is not None:
                    min=feature_xml_data.get("pMin")
                elif feature_xml_data.get("Min") is not None:
                    min=feature_xml_data.get("Min", "notDefined")
                if feature_xml_data.get("pMax") is not None:
                    max=feature_xml_data.get("pMax")
                elif feature_xml_data.get("Max") is not None:
                    max=feature_xml_data.get("Max", "notDefined")
                
                access_mode = feature_xml_data.get("AccessMode", "NA")
                value = "notReadable"
                
                # Enum options should be parsed regardless of read access
                if feature_type == "Enumeration":
                    options = []
                    enum_entries = feature_xml_data.get("EnumEntry", [])
                    if not isinstance(enum_entries, list):
                        enum_entries = [enum_entries]
                    for option in enum_entries:
                        options.append(option.get("@Name"))
                    details["options"] = options

                if access_mode in ["RO", "RW", "NA"]:
                    if feature_type == "Boolean":
                        try:
                            value = device.get_boolean_feature_value(feature_name) if device.get_boolean_feature_value(feature_name) is not None else "notReadable"
                        except Exception as e:
                            print(f"Warning: Could not get value for Boolean feature '{feature_name}' in '{group_or_category_name}': {e}")
                            value = "notReadable"
                    elif feature_type == "Enumeration":
                        try:
                            value = device.get_string_feature_value(feature_name) if device.get_string_feature_value(feature_name) is not None else "notReadable"
                        except Exception as e:
                            print(f"Warning: Could not get value for Enumeration feature '{feature_name}' in '{group_or_category_name}': {e}")
                            value = "notReadable"
                    elif feature_type == "Integer":
                        representation = feature_xml_data.get("Representation", "int")
                        try:
                            value = device.get_integer_feature_value(feature_name) if device.get_integer_feature_value(feature_name) is not None else "notReadable"
                        except Exception as e:
                            print(f"Warning: Could not get value for Integer feature '{feature_name}' in '{group_or_category_name}': {e}")
                            value = "notReadable"
                        details["min"] = min
                        details["max"] = max
                        details["representation"] = representation
                    elif feature_type == "Float":
                        details["min"] = min
                        details["max"] = max
                    elif feature_type == "String":
                        try:
                            value = device.get_string_feature_value(feature_name) if device.get_string_feature_value(feature_name) is not None else "notReadable"
                        except Exception as e:
                            print(f"Warning: Could not get value for String feature '{feature_name}' in '{group_or_category_name}': {e}")
                            value = "notReadable"
                else:
                    # If not readable, we keep the value as "notReadable"
                    print(f"Info: Feature '{feature_name}' is not readable (AccessMode: {access_mode}). Skipping read.")
                
                details.update({
                    "value": value,
                    "type": feature_type,
                    "tooltip": tooltip,
                    "description": description,
                })
                return details

            # --- Step 1: Build a comprehensive map of all top-level features ---
            self.all_features_map = {}
            # Exclude 'Command', 'Register', 'MaskedIntReg' as they are not simple features with values
            feature_types = ['Integer', 'Float', 'Boolean', 'Enumeration', 'String']
            for f_type in feature_types:
                features_of_type = main.get(f_type)
                if features_of_type:
                    if not isinstance(features_of_type, list):
                        features_of_type = [features_of_type]
                    for feature in features_of_type:
                        feature_name = feature.get("@Name")
                        if feature_name:
                            # Store the entire feature dictionary and its type for later use
                            self.all_features_map[feature_name] = {"definition": feature, "type": f_type}
            
            # --- Step 2: Process Groups ---
            groups=main.get("Group")    
            parsed_features={} # Temporarily store group features
            for group in groups:
                try:
                    group_name=group.get("@Comment")
                except:
                    print("Group has no @Comment, skipping...")
                    continue
                print("---------------"+group_name+"------------------")
                feature_temp={}
                # Helper function to get feature value and details

                # Process Boolean features
                boolean_features = group.get("Boolean")
                if boolean_features is not None:
                    if not isinstance(boolean_features, list): boolean_features = [boolean_features]
                    for bool_feature in boolean_features:
                        bool_name = bool_feature.get("@Name")
                        if bool_name:
                            feature_details = get_feature_details(bool_name, "Boolean", bool_feature, group_name, category={})
                            feature_temp[bool_name] = feature_details
                            self.features["booleans"].append({"name": bool_name, "group": group_name})
                            print(f"Boolean: {bool_name} = {feature_temp[bool_name]['value']}")

                # Process Enumeration features
                enumerations = group.get("Enumeration")
                if enumerations is not None:
                    if not isinstance(enumerations, list): enumerations = [enumerations]
                    for enum_feature in enumerations:
                        enum_name = enum_feature.get("@Name")
                        if enum_name:
                            feature_details = get_feature_details(enum_name, "Enumeration", enum_feature, group_name, category={})
                            feature_temp[enum_name] = feature_details
                            self.features["enumerations"].append({"name": enum_name, "group": group_name})
                            print(f"Enumeration: {enum_name} = {feature_temp[enum_name]['value']} with options {feature_temp[enum_name].get('options')}")
                
                # Process Integer features
                integers = group.get("Integer")
                if integers is not None:
                    if not isinstance(integers, list): integers = [integers]
                    for int_feature in integers:
                        int_name = int_feature.get("@Name")
                        if int_name:
                            feature_details = get_feature_details(int_name, "Integer", int_feature, group_name, category={})
                            feature_temp[int_name] = feature_details
                            self.features["integers"].append({"name": int_name, "group": group_name})
                            print(f"Integer: {int_name} = {feature_temp[int_name]['value']}")
                
                # Process Float features
                floats = group.get("Float")
                if floats is not None:
                    if not isinstance(floats, list): floats = [floats]
                    for float_feature in floats:
                        float_name = float_feature.get("@Name")
                        if float_name:
                            feature_details = get_feature_details(float_name, "Float", float_feature, group_name, category={})
                            feature_temp[float_name] = feature_details
                            self.features["floats"].append({"name": float_name, "group": group_name})
                            print(f"Float: {float_name} = {feature_temp[float_name]['value']}")
                
                # Process String features
                strings = group.get("String")
                if strings is not None:
                    if not isinstance(strings, list): strings = [strings]
                    for string_feature in strings:
                        string_name = string_feature.get("@Name")
                        if string_name:
                            feature_details = get_feature_details(string_name, "String", string_feature, group_name, category={})
                            feature_temp[string_name] = feature_details
                            self.features["strings"].append({"name": string_name, "group": group_name})
                            print(f"String: {string_name} = {feature_temp[string_name]['value']}")
                
                # print("---------------"+group_name+"------------------")
                # pp.pprint(feature_temp)
                if((feature_temp)!={}):
                    # if(group_name == "DeviceControl"):
                    #     self.deviceStats=feature_temp
                    # else:
                        # Store group features temporarily with source metadata
                        group_features_with_source = {
                            "source": "group",
                            "features": {}
                        }
                        for feat_name, feat_details in feature_temp.items():
                            group_features_with_source["features"][feat_name] = feat_details
                        parsed_features[group_name] = group_features_with_source # Store with source

            # --- Step 3: Process Categories ---
            categories = main.get("Category")
            if categories:
                if not isinstance(categories, list): categories = [categories]
                for category in categories:
                    category_name = category.get("@Name")
                    p_features = category.get("pFeature")
                    print(f"------------------ {category_name} --------------------")
                    if category_name and p_features:
                        if not isinstance(p_features, list): p_features = [p_features]
                        
                        category_features_data = {
                            "source": "category",
                            "features": {}
                        }
                        for p_feature_name in p_features:
                            feature_info = self.all_features_map.get(p_feature_name)
                            if feature_info:
                                feature_def = feature_info.get("definition")
                                feature_type = feature_info.get("type")
                                
                                if feature_def and feature_type:
                                    # Use the helper function to get structured details
                                    feature_details = get_feature_details(p_feature_name, feature_type, feature_def, category_name, category)
                                    category_features_data["features"][p_feature_name] = feature_details
                                    self.features[f"{feature_type.lower()}s"].append({"name": p_feature_name, "group": category_name})
                                    print(f"Category Feature - {feature_type}: {p_feature_name} = {feature_details['value']}")
                        if category_features_data:
                            self.feature_categories[category_name] = category_features_data
                            parsed_features[category_name] = category_features_data # Also add to parsed_features for merging
                            # print(f"--- Category: {category_name} ---")
                            # pp.pprint(category_features_data)
                            # print("--------------------------")


            # # Add features from categories as new groups, adding source metadata
            # if self.feature_categories:
            #     for category_name, features_in_category in self.feature_categories.items():
            #         category_data_with_source = {}
            #         for feat_name, feat_details in features_in_category.items():
            #             feat_details["source"] = "category"
            #             category_data_with_source[feat_name] = feat_details
                    
            #         # Add category features as a new group.
            #         # If a category name clashes with an existing group name, this will overwrite.
            #         # A more robust solution might be to merge or rename. For now, let's assume distinct names or overwrite.
            #         parsed_features[category_name] = category_data_with_source
            
            self.feature_groups = parsed_features # Assign the merged structure
            with open("camera_features.json", "w") as json_file:
                json.dump(self.feature_groups, json_file, indent=4)
            print("Camera features saved to 'camera_features.json'.")
            
        except Exception as e:
            print(f"Error retrieving GenICam XML: {e}")
            return None


# if __name__ == "__main__":
#     parser = argparse.ArgumentParser(description="Interactive tool for Xenics thermal cameras.")
#     parser.add_argument("camera_ip_or_name", nargs='?', default=CAMERA_NAME,
#                         help=f"The IP/Name of the Xenics camera. Default: '{CAMERA_NAME}'")
#     args = parser.parse_args()
    
#     # Import GLib hier, um sicherzustellen, dass es vor der Verwendung importiert wird
#     try:
#         from gi.repository import GLib
#     except ImportError:
#         print("Error: GLib not found. Please ensure PyGObject is installed correctly.")
#         sys.exit(1)

#     tool = XenicsInteractiveTool(args.camera_ip_or_name)
