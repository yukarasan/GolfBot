import requests

def main():
    try:
        response = requests.get('http://localhost:8081/')

        # Check if the request was successful
        if response.status_code == 200:
            # Print the response content
            json_data = response.json()

            # Print the JSON response
            print(json_data)
        else:
            print('Request failed.')

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()