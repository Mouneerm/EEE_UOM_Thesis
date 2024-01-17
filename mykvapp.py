# -*- coding: utf-8 -*-
"""
Created on Sat Jun 18 17:19:36 2022

@author: Mouneer Mahomed
"""
from kivy.config import Config
Config.set('graphics', 'resizable', True)
import kivy
from kivy.app import App
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.spinner import Spinner
from kivy_garden.mapview import MapView, MapSource, MapMarker
from kivy.clock import Clock

import pyrebase 

fb_gps_config = {
  "apiKey": "AIzaSyDonAyVFC0sdifr2ClNhC2jthntGGWN7xo",
  "authDomain": "diss-7c211.firebaseapp.com",
  "databaseURL": "https://diss-7c211-default-rtdb.firebaseio.com",
  "projectId": "diss-7c211",
  "storageBucket": "diss-7c211.appspot.com",
  "messagingSenderId": "351049480387",
  "appId": "1:351049480387:web:800feff7c8b57157d7ca76",
  "measurementId": "G-PC260DBP3T"
}

fb_gps =  pyrebase.initialize_app(fb_gps_config)
db_gps = fb_gps.database()

bus_routes = ("15 Flacq", "15 Rose-Hill", "16 Flacq", "16 Rose-Hill","194 Port-Louis", "194 Rose-Hill")
 
global selected_bus_route
selected_bus_route = ""
class myapp(App):
    
    
    def build(self):
        
        app = App.get_running_app()
        app.selected_bus_route = ""
        
        layout = GridLayout(rows = 2)

        app.map_view = MapView(lat = -20.17 , lon = 57.55 , zoom = 10) #initial lat lon mauritius
        app.map_view.map_source = "osm"
        app.map_marker = MapMarker() ######app here
        
        def show_selected_value(spinner, text):
            
            app.selected_bus_route = text
            #print("Bus route selected: ", app.selected_bus_route)            
            
  
            Clock.schedule_interval(self.update, 1)
                


                
        self.SpinnerObj = Spinner(text="Select your bus route here! ", values=bus_routes)
        
        layout.add_widget(self.SpinnerObj)
        self.SpinnerObj.bind(text = show_selected_value)
        
        #print(app.map_view)
    
        layout.add_widget(app.map_view)
        
        return layout
    
    def update(self, *args):
        app = App.get_running_app()

        app.map_view.remove_widget(app.map_marker) 
        #print(app.selected_bus_route)
        if app.selected_bus_route != "":
               
        
            gps_coord = db_gps.child("GPS").child(app.selected_bus_route).get()
            app.map_marker.lon = gps_coord.val()['lon']
            #print(app.map_marker.lon)
            app.map_marker.lat= gps_coord.val()['lat']
            #print(app.map_marker.lat)
        #map_marker.source = "marker.png"
            if app.map_marker.lon != 0.0  and app.map_marker.lat != 0.0: 
                app.map_view.add_widget(app.map_marker)
                

                
if __name__ == "__main__":
    myapp().run()
    
    
