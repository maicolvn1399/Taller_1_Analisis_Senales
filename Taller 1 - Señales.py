import numpy as np
from matplotlib import pyplot as plt
import math

def phasor_values(): 
    
    while(True):
        print("Ingrese valores")
        voltage_source = float(input("Ingrese valor VS [V]: "))
        voltage_resistance = float(input("Ingrese valor VR [V]: "))
        voltage_impedance = float(input("Ingrese valor VZL [V]: "))
        resistance_value = float(input("Ingrese valor de R [Ohms]: "))

        if(voltage_source > voltage_resistance + voltage_impedance):
            print("No es válido estos datos, vuelva a intentar")
        else:
            phasor_calculations(voltage_source, voltage_resistance, voltage_impedance, resistance_value)
            break

def phasor_calculations(vs, vr, vzl, res_val):
    theta = np.linspace(0, 2 * np.pi, 100)

    x1 = vr * np.cos(theta)
    y1 = vr * np.sin(theta)

    x2 = vzl * np.cos(theta) + vs
    y2 = vzl * np.sin(theta)

    # grafico de circunferencias
    plt.figure(figsize=(6, 6))
    plt.plot(x1, y1)
    plt.plot(x2,y2)

    #puntos de intersecciones 
    first_intercept, second_intercept = get_intersections_in_circumferences(0,0,vr,vs,0,vzl)
    plt.scatter(first_intercept, second_intercept, color='black', zorder=5, label='Intersecciones')

    #vector VS de (1,0)
    plt.quiver(0, 0, vs, 0, angles='xy', scale_units='xy', scale=1, color='red', label='Vector VS')

    # vector VR
    # orden correcto de intersecciones
    vrx1 = first_intercept[0]
    vry1 = second_intercept[1]
    #print(vrx1)
    #print(vry1)

    vector_VS = np.array([vs,0])
    vector_VR = np.array([vrx1,vry1])

    dot_product_VR_VS = get_dot_product(vector_VS,vector_VR)
    magnitude_VR = get_magnitude(vector_VR)
    magnitude_VS = get_magnitude(vector_VS)
    angle_VS_VR = get_angle(dot_product_VR_VS,magnitude_VS, magnitude_VR)
    #print(str(angle_VS_VR) + " degrees") 

    plt.quiver(0, 0, vrx1, vry1, angles='xy', scale_units='xy', scale=1, color='purple', label=f'Vector VR = {magnitude_VR}')


    draw_angle_VS_VR = np.linspace(0, degrees_to_radians(angle_VS_VR), 100)
    x_arc_VS_VR = np.cos(draw_angle_VS_VR) * magnitude_VS / 4
    y_arc_VS_VR = np.sin(draw_angle_VS_VR) * magnitude_VS / 4
    plt.plot(x_arc_VS_VR, y_arc_VS_VR, 'g', label=f'Angulo VS_VR = {angle_VS_VR}°')

    # Vector VZL
    vector_VZL = np.array([vs - vrx1, 0 - vry1])
    magnitude_VZL = get_magnitude(vector_VZL)
    #print("Vector VZL")
    #print(magnitude_VZL)
    plt.quiver(vrx1, vry1, vs - vrx1, 0 - vry1, angles='xy', scale_units='xy', scale=1, color='green', label=f'Vector VZL = {magnitude_VZL}')
    

    
    slope = get_slope((0,vry1),(vs,vrx1))
    y_intercept = get_y_intercept_of_line(slope, (vrx1,vry1))

    # valores de x
    x_values = np.linspace(-10, 10, 400)
    y_values = slope * x_values + y_intercept

    plt.plot(x_values, y_values, color='pink', linestyle='--')

    #Perpendicular line 
    slope_perpendicular = -1/slope
    y_values_perpendicular = slope_perpendicular * x_values
    plt.plot(x_values, y_values_perpendicular, color='yellow', linestyle='--')

    #intercept between lines 
    intercept_in_lines = get_intercept_between_lines(slope, y_intercept, slope_perpendicular, 0)
    plt.scatter(intercept_in_lines[0], intercept_in_lines[1], color='black', zorder=5)


    #Vectores de corriente
    #ICZL

    vector_ICZL = np.array([intercept_in_lines[0], intercept_in_lines[1]])

    dot_product_VS_ICZL = get_dot_product(vector_VS, vector_ICZL)
    magnitude_ICZL = get_magnitude(vector_ICZL)
    angle_VS_ICZL = get_angle(dot_product_VS_ICZL, magnitude_VS, magnitude_ICZL)
    plt.quiver(0, 0, intercept_in_lines[0], intercept_in_lines[1], angles='xy', scale_units='xy', scale=1, color='brown', label=f'Vector ICZL = {magnitude_ICZL}')
    #print(str(angle_VS_ICZL) + " degrees")

    draw_angle_VS_ICZL = np.linspace(0, degrees_to_radians(angle_VS_ICZL), 100)
    arc_radius = min(magnitude_VS, magnitude_ICZL) / 2
    x_arc_VS_ICZL = np.cos(draw_angle_VS_ICZL) * arc_radius
    y_arc_VS_ICZL = np.sin(draw_angle_VS_ICZL) * arc_radius
    plt.plot(x_arc_VS_ICZL, y_arc_VS_ICZL, 'r', label=f'Angulo VS_ICZL  {angle_VS_ICZL}°')

    #y = 0,3039
    x_line = np.linspace(-10, 10, 400)  

    vector_y_line = np.array([0, intercept_in_lines[1]])
    y_line = np.full_like(x_line,  intercept_in_lines[1])
    plt.plot(x_line, y_line, label=f'y = {intercept_in_lines[1]}',color='cyan', linestyle='--')

    angulo_radianes = np.arctan2(vector_VZL[1], vector_VZL[0]) - np.arctan2(vector_y_line[1], vector_y_line[0])

    # Convertir el ángulo a grados y hacerlo negativo
    angulo_grados = np.degrees(angulo_radianes)
    

    #IRZL
    
    vector_IRZL = np.array([vrx1 - intercept_in_lines[0], vry1 - intercept_in_lines[1]])
    magnitude_IRZL = get_magnitude(vector_IRZL)
    plt.quiver(intercept_in_lines[0], intercept_in_lines[1],vrx1 - intercept_in_lines[0],vry1 - intercept_in_lines[1] ,angles='xy', scale_units='xy', scale=1, color='magenta', label=f'Vector IRZL = {magnitude_IRZL}')

    plt.title('Phasors')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis('equal')  #todos los ejes sean iguales 
    plt.grid(True)
    plt.legend()
    plt.show()

def degrees_to_radians(degrees):
    return np.deg2rad(degrees)

def get_angle(dot_product, magnitude1, magnitude2):

    cos_theta = dot_product / (magnitude1 * magnitude2)

    # angulo en radianes
    angle_radians = np.arccos(cos_theta)

    # se convierte el angulo a grados
    angle_degrees = np.degrees(angle_radians)

    return angle_degrees


def get_magnitude(vector):
    magnitude = np.linalg.norm(vector)
    return magnitude


def get_dot_product(vector1, vector2):
    dot_product = np.dot(vector1,vector2)
    return dot_product


def get_intercept_between_lines(slope1, y_intercept1, slope2, y_intercept2):
    if slope1 == slope2:
        if y_intercept1 == y_intercept2:
            return "lineas son iguales."
        else:
            return "lineas son paralelas y no intersecan"
    
    # Calcular coordenada x de la interseccion
    x = (y_intercept2 - y_intercept1) / (slope1 - slope2)
    
    # Calcular coordenada y de la interseccion
    y = slope1 * x + y_intercept1
    
    return (x, y)

def get_slope(inters_y, inters_x):
    y2,y1 = inters_y
    x2,x1 = inters_x
  
    slope = (y2 - y1) / (x2 - x1)
    return slope

def get_y_intercept_of_line(slope, point):
    x1, y1 = point
    # y = mx + b => b = y - mx
    y_intercept = y1 - slope * x1
    return y_intercept

def get_intersections_in_circumferences(x0, y0, r0, x1, y1, r1):
    # circulo 1: (x0, y0), radio r0
    # circulo 2: (x1, y1), radio r1

    d=math.sqrt((x1-x0)**2 + (y1-y0)**2)
    
    # cuando no intersecan
    if d > r0 + r1 :
        return None
    # cuando un circulo esta dentro de otro 
    if d < abs(r0-r1):
        return None
    # coinciden los circulos 
    if d == 0 and r0 == r1:
        return None
    else:
        a=(r0**2-r1**2+d**2)/(2*d)
        h=math.sqrt(r0**2-a**2)
        x2=x0+a*(x1-x0)/d   
        y2=y0+a*(y1-y0)/d   
        x3=x2+h*(y1-y0)/d     
        y3=y2-h*(x1-x0)/d 

        x4=x2-h*(y1-y0)/d
        y4=y2+h*(x1-x0)/d

        #print((x3, y3), (x4, y4))
        
        return (x3, x4), (y3, y4)


phasor_values()

