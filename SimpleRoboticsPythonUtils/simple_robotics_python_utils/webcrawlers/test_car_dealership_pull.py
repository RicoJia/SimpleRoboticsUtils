#!/usr/bin/env/ python3

"""
TODO: this script can be checked on 
https://www.toyota.com/search-inventory/model/rav4hybrid/?zipcode=77008&showCompareVehicle=false&distance=20&dealerDistance%5B%5D=42138%2C42087%2C42073%2C42270`
Joe Myers one is hard coded
"""

import requests
from bs4 import BeautifulSoup

from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.chrome.options import Options
import time
from PIL import Image
import os

import smtplib
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from email.mime.base import MIMEBase
from email import encoders
from email.mime.image import MIMEImage
import requests
from io import BytesIO

from fpdf import FPDF

from dataclasses import dataclass
from typing import List, Callable


@dataclass
class Dealership:
    name: str
    url: str
    parse_func: Callable


@dataclass
class Car:
    model: str
    price: str
    trim: str
    url: str
    image_url: str
    dealership: Dealership


def save_cars_to_pdf(cars: List[Car], file_path: str):

    # Initialize the PDF
    pdf = FPDF()
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.add_page()
    pdf.set_font("Arial", size=12)
    y_position = 10  # Start position at the top of the page

    image_files = []

    for i, car in enumerate(cars):
        # Add car information
        pdf.set_font("Arial", style="B", size=14)

        pdf.set_xy(10, y_position)
        pdf.cell(0, 10, txt=f"{car.dealership.name} - {car.model} - {car.trim}", ln=True, link=car.url)
        y_position += 10  # Move down after title

        pdf.set_font("Arial", size=12)
        pdf.set_xy(10, y_position)
        pdf.cell(0, 10, txt=f"Price: {car.price}", ln=True)
        y_position += 10

        # Download and add the image
        response = requests.get(car.image_url)
        if response.status_code == 200:
            image_data = BytesIO(response.content)
            image = Image.open(image_data)

            # Convert to RGB if needed
            if image.mode == "RGBA":
                image = image.convert("RGB")

            # Save temporarily and add to PDF
            image_path = f"temp_car_image_{i}.jpg"
            image.save(image_path)
            image_files.append(image_path)

            # Add the image
            pdf.image(image_path, x=10, y=y_position, w=100)
            y_position += 60  # Adjust based on image height

        # Add spacing between entries
        y_position += 10

        # Check if we need a new page
        if y_position > 260:  # Approximate bottom of the page
            pdf.add_page()
            y_position = 10  # Reset position for new page

    # Save the PDF
    pdf.output(file_path)
    print(f"PDF saved as {file_path}")

    for f in image_files:
        os.remove(f)


def parse_don_mcgill(soup, dealership, cars):
    listings = soup.find_all("div", class_="listing")

    for listing in listings:
        # Get the model (from `data-ag-model` attribute)
        model = listing.get("data-ag-model", "N/A")
        trim = listing.get("data-ag-trim", "N/A")
        if "Premium" in trim:

            price = listing.get("data-ag-price", "N/A")
            # Get the relative URL (nested under `div.listing-container > data-url`)
            container = listing.find("div", class_="listing-container")
            sub_url = container.get("data-url", "N/A") if container else "N/A"

            image_div = listing.find("img", class_="srp-click-image")  # Locate the image
            image_url = image_div.get("data-src", "N/A") if image_div else "N/A"

            # Print the extracted information
            model_url = f"{dealership.url}{sub_url}"
            print(f"Model: {model}")
            print(f"Price: {price}")
            print(f"trim: {trim}")
            print(f"")
            print("-" * 40)
            print(f"Image URL: {image_url}")
            car = Car(model=model, price=price, trim=trim, image_url=image_url, url=model_url, dealership=dealership)
            cars.append(car)


def parse_joe_myers(soup, dealership, cars):
    listings = soup.find_all("li", class_="box box-border vehicle-card vehicle-card-detailed")
    for listing in listings:
        image_tag = listing.find("img")
        image_url = image_tag["src"] if image_tag else "N/A"
        # Extract model and trim
        title_tag = listing.find("h2", class_="vehicle-card-title")
        model_trim = title_tag.get_text(strip=True) if title_tag else "N/A"
        # Extract price
        price_tag = listing.find("dd", class_="final-price")
        price = (
            price_tag.find("span", class_="price-value").get_text(strip=True)
            if price_tag and price_tag.find("span", class_="price-value")
            else "N/A"
        )
        if image_tag is None:
            continue
        print(f"Image URL: {image_url}")
        print(f"Model and Trim: {model_trim}")
        print(f"Price: {price}")
        car = Car(model="", price=price, trim=model_trim, image_url=image_url, url="", dealership=dealership)
        cars.append(car)


dealerships = [
    Dealership(
        name="Don Mcgill", url="https://www.donmcgilltoyota.com/new-vehicles/?q=rav4%20hybrid", parse_func=parse_don_mcgill
    ),
    # TODO: 3 pages based on experience
    Dealership(
        name="Joe Myers",
        url="https://www.joemyerstoyota.com/new-inventory/index.htm?make=Toyota&model=RAV4%20Hybrid",
        parse_func=parse_joe_myers,
    ),
    Dealership(
        name="Joe Myers",
        url="https://www.joemyerstoyota.com/new-inventory/index.htm?make=Toyota&model=RAV4%20Hybrid&start=16",
        parse_func=parse_joe_myers,
    ),
    Dealership(
        name="Joe Myers",
        url="https://www.joemyerstoyota.com/new-inventory/index.htm?make=Toyota&model=RAV4%20Hybrid&start=32",
        parse_func=parse_joe_myers,
    ),
]

chrome_options = Options()
chrome_options.add_argument("--headless")  # Enable headless mode
chrome_options.add_argument("--no-sandbox")  # Avoid sandboxing (useful on Linux servers)
chrome_options.add_argument("--disable-dev-shm-usage")  # Prevent shared memory issues on Linux
driver = webdriver.Chrome(options=chrome_options)  # Ensure the Chrome driver is installed and in PATH
cars = []
for dealership in dealerships:

    driver.get(dealership.url)
    # Wait for the page to load fully
    time.sleep(5)
    # driver.implicitly_wait(10)
    html = driver.page_source
    soup = BeautifulSoup(html, "html.parser")
    dealership.parse_func(soup=soup, dealership=dealership, cars=cars)

driver.quit()
save_cars_to_pdf(cars, "car_report.pdf")
