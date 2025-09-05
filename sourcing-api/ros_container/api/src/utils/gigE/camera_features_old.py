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
import logging # Import logging
gi.require_version('Aravis', '0.8')
from gi.repository import Aravis

logger = logging.getLogger(__name__) # Initialize logger

class XenicsInteractiveTool:
    def __init__(self, camera_identifier):
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
        try:
            print(f"Connecting to camera: '{camera_identifier}'...")
            self.camera = Aravis.Camera.new(camera_identifier)
            if self.camera is None:
                
                raise Exception(f"Camera '{camera_identifier}' not found.")
            print(f"Connected to {self.camera.get_model_name()} ({self.camera.get_device_id()})")
            self.dump_genicam_xml()
            logger.debug(f"Feature groups after dump_genicam_xml: {self.feature_groups}") # Debug log
        except Exception as e:
            print(f"Error: Could not connect to camera. Details: {e}")
            sys.exit(1)
        
        # Safely get width, height, name, and IP address
        self.width = self.feature_groups.get("ImageFormatControl", {}).get("Width", {}).get("value")
        self.height = self.feature_groups.get("ImageFormatControl", {}).get("Height", {}).get("value")
        self.name = f"{self.camera.get_model_name()} ({self.camera.get_device_id()})"
        self.ip_address = self.feature_groups.get("DeviceInformation", {}).get("GevDeviceIPAddress", {}).get("value")

    def dump_genicam_xml(self):
        """Dumps the GenICam XML from the camera."""
        try:
            device = self.camera.props.device
            genicam_xml = device.get_genicam_xml()
            print("Successfully retrieved GenICam XML.")
            pp = pprint.PrettyPrinter(indent=4)
            # pp.pprint(json.dumps(xmltodict.parse(genicam_xml[0])))
            xml_dict=xmltodict.parse(genicam_xml[0], force_list=('String', 'Enumeration'))
            main=xml_dict.get("RegisterDescription")
            groups=main.get("Group")    
            parsed_features={}
            for group in groups:
                try:
                    group_name=group.get("@Comment")
                except:
                    print("Group has no @Comment, skipping...")
                    continue
                print("---------------"+group_name+"------------------")
                feature_temp={}
                if(group.get("Boolean"!=None and type(group.get("Boolean", None)) is list)):
                    for bool in group.get("Boolean", {}):
                        bool_name=bool.get("@Name")
                        tooltip=bool.get("Tooltip", "")
                        description=bool.get("Description", tooltip)
                        value= device.get_boolean_feature_value(bool_name) if device.get_boolean_feature_value(bool_name) != None else "notReadable"
                        feature_temp[bool_name]={
                            "value": value,
                            "type": "Boolean",
                            "tooltip": tooltip,
                            "description": description
                        }
                        self.features["booleans"].append({"name":bool_name, "group":group_name})
#                        feature_temp["datatypes"]["booleans"].append(bool_name)
                        print(f"Boolean: {bool_name} = {value}")

                # Iterate over Enumeration entries, assuming group.get("Enumeration", []) returns a list of dictionaries,
                # where each dictionary has a single key (the enumeration name) and the value is the enumeration details.
                if(group.get("Enumeration", None) !=None and type(group.get("Enumeration", None)) is list):
                    for enumeration_dict in group.get("Enumeration", []):
                        # Extract the enumeration name (which is the key of the dictionary)
                        if enumeration_dict: # Ensure the dictionary is not empty
                            enum_name = enumeration_dict.get("@Name")
                        else:
                            continue # Skip if the enumeration dictionary is empty

                        options=[]
                        # Ensure EnumEntry is treated as a list, even if it's a single item
                        enum_entries = enumeration_dict.get("EnumEntry", [])
                        if not isinstance(enum_entries, list):
                            enum_entries = [enum_entries]
                        try:    
                            for option in enum_entries:
                                options.append(option.get("@Name"))
                            feature_temp[enum_name]={
                                "value": device.get_string_feature_value(enum_name) if device.get_string_feature_value(enum_name) != None else "notReadable",
                                "type": "Enumeration",
                                "options": options,
                                "tooltip": enumeration_dict.get("Tooltip", ""),
                                "description": enumeration_dict.get("Description", enumeration_dict.get("Tooltip", ""))
                            }
                            self.features["enumerations"].append({"name":enum_name, "group":group_name})
                      #      feature_temp["datatypes"]["enumerations"].append(enum_name)
                            print(f"Enumeration: {enum_name} = {feature_temp['features'][enum_name]['value']} with options {options}")
                        except Exception as e:
                            print("error in group "+group_name)
                            print(f"  --> Could not read enumeration feature '{enum_name}': {e}")
                            print("  --> Skipping this feature.")
                if(group.get("Integer", None) is not None and type(group.get("Integer", None)) is list):
                    for integer in group.get("Integer", {}):
                        try:
                            int_name=integer.get("@Name")
                            int_min=int(integer.get("@Min")) if integer.get("@Min")!=None else "notDefined"
                            int_max=int(integer.get("@Max")) if integer.get("@Max")!=None else "notDefined"
                            tooltip=integer.get("Tooltip", "")
                            description=integer.get("Description", tooltip)
                            value=0
                            representation="int"
                            if(integer.get("Representation", None)!=None and integer.get("Representation") == "HexNumber"):
                                value=device.get_string_feature_value(int_name) if device.get_string_feature_value(int_name) != None else "notReadable"
                                print(f"HexNumber: {int_name} = {value}")
                                representation="hex"
                            else:
                                value=device.get_integer_feature_value(int_name) if device.get_integer_feature_value(int_name) != None else "notReadable"
                                print(f"Integer: {int_name} = {value}")
                            feature_temp[int_name]={
                                "value": value,
                                "type": "Integer",
                                "min": int_min,
                                "max": int_max, 
                                "tooltip": tooltip,
                                "representation": representation,
                                "description": description
                            }
                            self.features["integers"].append({"name":int_name, "group":group_name})
                     #       feature_temp["datatypes"]["integers"].append(int_name)
                        except Exception as e:
                            print("error in group "+group_name)
                            print(f"  --> Could not read integer feature '{int_name}': {e}")
                            print("  --> Skipping this feature.")
                if(group.get("Float", None) is not None and type(group.get("Float", None)) is list):
                    for float_ in group.get("Float", {}):
                        try:
                            float_name=float_.get("@Name")
                            float_min=float(float_.get("@Min")) if float_.get("@Min")!=None else "notDefined"
                            float_max=float(float_.get("@Max")) if float_.get("@Max")!=None else "notDefined"
                            tooltip=float_.get("Tooltip", "")
                            description=float_.get("Description", tooltip)
                            value=device.get_float_feature_value(float_name) if device.get_float_feature_value(float_name) != None else "notReadable"
                            feature_temp[float_name]={
                                "value": value,
                                "type": "Float",
                                "min": float_min,
                                "max": float_max, 
                                "tooltip": tooltip,
                                "description": description
                            }
                            #feature_temp["datatypes"]["floats"].append(float_name)
                            self.features["floats"].append({"name":float_name, "group":group_name})
                            print(f"Float: {float_name} = {value}")
                        except Exception as e:
                            print("error in group "+group_name)
                            print(f"  --> Could not read float feature '{float_name}': {e}")
                            print("  --> Skipping this feature.")
                if(group.get("String", None) is not None and type(group.get("String", None)) is list):
                    try:
                        for string in group.get("String", {}):
                            string_name=string.get("@Name")
                            tooltip=string.get("Tooltip", "")
                            description=string.get("Description", tooltip)
                            value=device.get_string_feature_value(string_name) if device.get_string_feature_value(string_name) != None else "notReadable"
                            feature_temp[string_name]={
                                "value": value,
                                "type": "String",
                                "tooltip": tooltip,
                                "description": description
                            }
                            self.features["strings"].append({"name":string_name, "group":group_name})
                            print(f"String: {string_name} = {value}")
                    except Exception as e:
                        print("error in group "+group_name)
                        print(f"  --> Could not read string feature '{string_name}': {e}")
                        print("  --> Skipping this feature.")
                print("---------------"+group_name+"------------------")
                pp.pprint(feature_temp)
                if((feature_temp)!={}):
                    if(group_name == "DeviceControl"):
                        self.deviceStats=feature_temp
                    else:
                        parsed_features[group_name]=feature_temp
            self.feature_groups=parsed_features
        except Exception as e:
            print(f"Error retrieving GenICam XML: {e}")
            return None
