def calculate_position(obj):
    # Exemple de fonction de calcul simplifiée
    x, y, w, h = obj["coordinates"]
    azimuth = x * 0.1  # Exemple de calcul
    elevation = y * 0.1
    distance = (w + h) * 0.5
    return {"azimuth": azimuth, "elevation": elevation, "distance": distance}
