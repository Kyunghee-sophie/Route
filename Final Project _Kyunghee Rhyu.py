from numpy import *
import matplotlib.pyplot as plt
import descartes
from geopandas import *
import earthpy as et
from shapely.geometry import Point, Polygon
from pandas import *
import fiona
import os
import osmnx as ox
import networkx as nx
from sklearn.neighbors import KDTree
import adjustText as aT

%matplotlib inline

# read data
incidents = pd.read_csv("C:\\DFR\\DFR_INCIDENT_2015.csv")
stations = pd.read_csv("C:\\DFR\\DFR_STATION.csv")
GPS2015 = pd.read_csv("C:\\DFR\\DFR_2015_AVL_modified.csv")

wgs84 = {'init': 'epsg:4326'}

#Creating a GeoDataFrame from a DataFrame with coordinates
Gps2015 = geopandas.GeoDataFrame(GPS2015,# specify data
                                 crs=wgs84, # specify coordinate reference system
                                 geometry=geopandas.points_from_xy(GPS2015.Longitude, GPS2015.Latitude))# specify the geometry list 
Incidents = geopandas.GeoDataFrame(incidents, crs=wgs84,geometry=geopandas.points_from_xy(incidents.x, incidents.y)) 
Stations = geopandas.GeoDataFrame(stations, crs=wgs84, geometry=geopandas.points_from_xy(stations.Lon, stations.Lat))

stations

def shortest_route (Incidents, Stations, Gps):
    # incident location
    num = input ("Enter number :")
    inc_num = int(num)
    target = Incidents[Incidents['Master_Incident_Number'] == inc_num ]
    dest_point = (target.iloc[0][6], target.iloc[0][5]) # create coordinate 
    
    # gps data of the emergency vehicle 
    gps = Gps2015[Gps2015['Master_Incident_Number'] == target.iloc[0][1]]
    er = gps.iloc[-1][1]
    
    # fire station location
    origin = Stations[Stations['ENGINE'].isin([er])|Stations['RESCUE'].isin([er])|Stations['PEAK RES'].isin([er])|Stations['TRUCK'].isin([er])|\
               Stations['BAT CH'].isin([er])|Stations['DEP CH'].isin([er])|Stations['OTHER'].isin([er])|Stations['OTHER.1'].isin([er])|\
               Stations['OTHER.2'].isin([er])|Stations['OTHER.3'].isin([er])]
    orig_point = (origin.iloc[0][5], origin.iloc[0][6]) # create coordinate 
    
    # draw the map
    ox.config(log_console=True, use_cache=True)
    
    # grabbing all nodes and edges within 3 kilometers from origin point
    G = ox.graph_from_point(orig_point, distance = 3000)
    
    # assinging nodes
    nodes, _ = ox.graph_to_gdfs(G)
    nodes.head()
    
    # find optimal nodes 
    tree = KDTree(nodes[['y', 'x']], metric='euclidean')
    station_idx = tree.query([orig_point], k=1, return_distance=False)[0] # k=1 nearest neighbors 
    incident_idx = tree.query([dest_point], k=1, return_distance=False)[0]
    
    # find the nearest nodes from each points
    closest_node_to_station = nodes.iloc[station_idx].index.values[0]
    closest_node_to_incident = nodes.iloc[incident_idx].index.values[0]
    
    # create the shortes route from fire station to incident point
    route = nx.shortest_path(G, closest_node_to_station,
                         closest_node_to_incident, weight='distance')
    
    fig, ax = ox.plot_graph_route(G, route, fig_height=20, 
                              fig_width=20, 
                              show=False, close=False, 
                              edge_color='grey',
                              orig_dest_node_color='green',
                              route_color='green')
    ax.scatter(orig_point[1], orig_point[0], c='yellow', s=100)
    ax.scatter(dest_point[1], dest_point[0], c='blue', s=100)
    
    # draw the gps for emergency car
    gps.plot(ax=ax, markersize = 20, color = "red", marker = "o")
    
    # labelling date and time of the gps points
    texts = []
    for x, y, label in zip(gps.geometry.x, gps.geometry.y, gps["Date_Time"]):
        texts.append(plt.text(x, y, label, fontsize = 8))
        
    aT.adjust_text(texts, force_points=0.5, force_text=0.8, expand_points=(1,1), expand_text=(1,1), 
               arrowprops=dict(arrowstyle="-", color='grey', lw=0.5))
    plt.show()

# example 
# 'Master_Incident_Number' = 2015225875
shortest(Incidents, Stations, Gps2015)
