import requests
from bs4 import BeautifulSoup

# URL della pagina da cui fare scraping
url = 'https://it.windfinder.com/weatherforecast/lago_albano'

# Effettua la richiesta HTTP alla pagina
response = requests.get(url)

# Controlla che la richiesta abbia avuto successo
if response.status_code == 200:
    # Analizza il contenuto HTML della pagina
    soup = BeautifulSoup(response.text, 'html.parser')

    # Trova la sezione che contiene le informazioni del vento e meteo
    forecast_table = soup.find('table', class_='weathertable')

    if forecast_table:
        rows = forecast_table.find_all('tr')

        # Estrae informazioni rilevanti da ogni riga
        for row in rows:
            time = row.find('td', class_='time').text if row.find('td', class_='time') else 'N/A'
            wind_speed = row.find('span', class_='wind-speed').text if row.find('span', class_='wind-speed') else 'N/A'
            wind_direction = row.find('td', class_='direction').text if row.find('td', class_='direction') else 'N/A'
            wind_gusts = row.find('span', class_='wind-gust').text if row.find('span', class_='wind-gust') else 'N/A'
            precipitation = row.find('td', class_='precip').text if row.find('td', class_='precip') else 'N/A'

            # Stampa i dati raccolti
            print(f"Orario: {time}, Vento: {wind_speed}, Direzione: {wind_direction}, Raffiche: {wind_gusts}, Precipitazioni: {precipitation}")
    else:
        print("Tabella delle previsioni non trovata.")
else:
    print(f"Errore durante la richiesta: {response.status_code}")
