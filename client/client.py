import requests

def main():
    try:
        response = requests.geqt('http://localhost:8081/')

        # Check if the request was successful
        if response.status_code == 200:
            # Print the response content
            print(response.text)
        else:
            print('Request failed.')

    except Exception as e:
        print(e)

if __name__ == "__main__":
    main()