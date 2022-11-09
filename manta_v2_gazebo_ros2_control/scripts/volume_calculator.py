#!/usr/bin/env python
from hashlib import new
import math


class BuoyancyCalculator(object):

    def __init__(self):
        print("Buoyancy Force Calculator Initialised...")

    def start_ask_loop(self):

        selection = "START"

        while selection != "Q":
            print("#############################")
            print("Select Geometry to Calculate Buoyancy Force:")
            print("[1]Sphere radius(r)")
            print("[2]Cylinder radius(r)")
            print("[3]Box dimensions(x_size*y_size*z_size)")
            print("[Q]END program")
            selection = input(">>")
            self.select_action(selection)

        print("InertialCaluclator Quit...Thank you")

    def select_action(self, selection):
        if selection == "1":
            mass = float(input("mass>>"))
            radius = float(input("radius>>"))
            fluid_density = float(input("fluid_density [kg/m^3]>>"))
            self.calculate_sphere_buoyancy_force(mass, radius, fluid_density)
            
        elif selection == "2":
            mass = float(input("mass>>"))
            radius = float(input("radius>>"))
            height = float(input("height>>"))
            fluid_density = float(input("fluid_density [kg/m^3]>>"))
            self.calculate_cylinder_buoyancy_force(mass, radius, height, fluid_density)
            
        elif selection == "3":
            mass = float(input("mass>>"))
            x_size = float(input("x_size>>"))
            y_size = float(input("y_size>>"))
            z_size = float(input("z_size>>"))
            
            fluid_density = float(input("fluid_density [kg/m^3]>>"))
            self.calculate_box_buoyancy_force(mass, x_size, y_size, z_size, fluid_density, gravity=9.81)
            
        elif selection == "Q":
            print("Selected Quit")
        else:
            print("Usage: Select one of the give options")

    def calculate_sphere_volume(self, m, r):
        volume = (4.0/3.0)*math.pi*math.pow(r,3)
        return volume
        
    def calculate_cylinder_volume(self, m, r, h):
        # volume = pi*r^2*h
        volume = math.pi*math.pow(r,2)*h
        return volume
        
    def calculate_cylinder_area(self, r, h):
        # area = 2*pi*r^2 + 2*pi*r*h
        area = 2*math.pi*math.pow(r,2) + 2*math.pi*r*h
        return area
        
    def calculate_box_volume(self,x, y, z):
        # Volume of box is x*y*z
        volume = x*y*z
        return volume
    
    def calculate_sphere_buoyancy_force(self, mass, radius, fluid_density, gravity=9.81):
        volume = self.calculate_sphere_volume(m=mass, r=radius)
        fluid_eq_mass = fluid_density * volume
        buoyancy_force = fluid_eq_mass * gravity
        sphere_weight = mass * gravity
        print("Volume Sphere ="+str(volume))
        print("Mass of Fluid for Sphere Volume Sphere, This should be the mass of sphere for neutral buoyancy ="+str(fluid_eq_mass))
        print("Buoyancy Force ="+str(buoyancy_force))
        print("Weight Force of Sphere ="+str(sphere_weight))
        
        final_weight_force = buoyancy_force - sphere_weight
        
        print("RESULT OF WEIGHT AND BUOYANCY FORCE =="+str(final_weight_force))
        if final_weight_force < 0:
            print("The Sphere is going to SINK")
        elif final_weight_force == 0:
            print("The Sphere is going to have NEUTRAL Buoyancy")
        else:
            print("The Sphere is going to FLOAT")
            
            
    def calculate_cylinder_buoyancy_force(self, mass, radius, height, fluid_density, gravity=9.81):
        
        volume = self.calculate_cylinder_volume(m=mass, r=radius, h=height)
        area = self.calculate_cylinder_area(r=radius, h=height)
        
        fluid_eq_mass = fluid_density * volume
        buoyancy_force = fluid_eq_mass * gravity
        cylinder_weight = mass * gravity
        print("Volume Cylinder ="+str(volume))
        print("Area Cylinder ="+str(area))
        print("Mass of Fluid for Cylinder Volume Cylinder, This should be the mass of sphere for neutral buoyancy ="+str(fluid_eq_mass))
        print("Buoyancy Force ="+str(buoyancy_force))
        print("Weight Force of Cylinder ="+str(cylinder_weight))
        
        final_weight_force = buoyancy_force - cylinder_weight
        
        print("RESULT OF WEIGHT AND BUOYANCY FORCE =="+str(final_weight_force))
        if final_weight_force < 0:
            print("The Cylinder is going to SINK")
        elif final_weight_force == 0:
            print("The Cylinder is going to have NEUTRAL Buoyancy")
        else:
            print("The Cylinder is going to FLOAT")
            
            
    def calculate_box_buoyancy_force(self, mass, x_size, y_size, z_size, fluid_density, gravity=9.81):
        
        volume = self.calculate_box_volume(x=x_size, y=y_size, z=z_size)
        
        fluid_eq_mass = fluid_density * volume
        buoyancy_force = fluid_eq_mass * gravity
        box_weight = mass * gravity
        print("Volume Box ="+str(volume))
        print("Mass of Fluid for Box Volume, This should be the mass of Box for neutral buoyancy ="+str(fluid_eq_mass))
        print("Buoyancy Force ="+str(buoyancy_force))
        print("Weight Force of Box ="+str(box_weight))
        
        final_weight_force = buoyancy_force - box_weight
        
        print("RESULT OF WEIGHT AND BUOYANCY FORCE =="+str(final_weight_force))

        
        volume = box_weight/(fluid_density * gravity)
        new_y = volume/(x_size*z_size)
        print("Volume:{}".format(volume))
        print("Box y:{}".format(new_y))

        if final_weight_force < 0:
            print("The Box is going to SINK")
        elif final_weight_force == 0:
            print("The Box is going to have NEUTRAL Buoyancy")
        else:
            print("The Box is going to FLOAT")
        


if __name__ == "__main__":
    inertial_object = BuoyancyCalculator()
    inertial_object.start_ask_loop()