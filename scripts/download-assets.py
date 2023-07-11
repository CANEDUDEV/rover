#!/usr/bin/env python3

import io
import os
import sys

from googleapiclient.discovery import build
from googleapiclient.errors import HttpError
from google.oauth2 import service_account

from googleapiclient.http import MediaIoBaseDownload

# Set up credentials with the right scope
credentials = None
root_folder = "."

if len(sys.argv) > 1:
    root_folder = str(sys.argv[1])

credentials_path = os.path.join(root_folder, ".credentials", "rover-gdrive-sa.json")
credentials = service_account.Credentials.from_service_account_file(
    str(credentials_path)
)
scoped_credentials = credentials.with_scopes("https://www.googleapis.com/auth/drive")

assets_folder = os.path.join(root_folder, "docs", "assets")
os.makedirs(assets_folder, exist_ok=True)

try:
    # Build the service object.
    service = build("drive", "v3", credentials=credentials)
    # Call the Drive v3 API
    results = (
        service.files()
        .list(
            corpora="drive",
            driveId="0AKkG7JBumTpWUk9PVA",
            includeItemsFromAllDrives=True,
            supportsAllDrives=True,
            q="trashed=false",
        )
        .execute()
    )

    for obj in results["files"]:
        if obj["mimeType"] == "application/vnd.google-apps.folder":
            continue

        filename = obj["name"]
        file = os.path.join(assets_folder, filename)

        if os.path.exists(file):
            print(f"{filename} already downloaded, skipping...")
            continue

        try:
            file_id = obj["id"]
            request = service.files().get_media(fileId=file_id)
            file_bytes = io.BytesIO()
            downloader = MediaIoBaseDownload(file_bytes, request)
            done = False

            while done is False:
                status, done = downloader.next_chunk()
                print(f"Downloading {filename}... {int(status.progress() * 100)}%")

        except HttpError as error:
            print(f"Error: {error}")
            exit(1)

        # Write to file
        with open(file, "wb") as f:
            f.write(file_bytes.getbuffer())

except HttpError as error:
    print(f"Error: {error}")
    exit(1)
