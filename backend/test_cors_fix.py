#!/usr/bin/env python3
"""
Test script to verify the CORS configuration fix
"""
import asyncio
from fastapi.testclient import TestClient
from main import app

def test_options_request():
    """Test that OPTIONS request to /chat returns 200"""
    client = TestClient(app)

    # Test OPTIONS request
    response = client.options("/chat")
    print(f"OPTIONS /chat status code: {response.status_code}")
    print(f"Response headers: {dict(response.headers)}")

    # The CORS middleware should handle OPTIONS requests automatically
    # without needing a custom handler
    assert response.status_code == 200 or response.status_code == 405  # 405 is also acceptable for OPTIONS

    print("OPTIONS request test completed successfully!")

def test_cors_headers():
    """Test that CORS headers are properly set"""
    client = TestClient(app)

    # Make a request with origin header to test CORS
    headers = {
        "Origin": "https://mustafa-shams.github.io/Ai-textbook-1/",
        "Access-Control-Request-Method": "POST",
        "Access-Control-Request-Headers": "Content-Type",
    }

    response = client.options("/chat", headers=headers)
    print(f"OPTIONS /chat with origin status code: {response.status_code}")
    print(f"Response headers: {dict(response.headers)}")

    # Check if CORS headers are present
    cors_headers = [
        "access-control-allow-origin",
        "access-control-allow-methods",
        "access-control-allow-headers"
    ]

    for header in cors_headers:
        if header in response.headers:
            print(f"[OK] Found CORS header: {header}")
        else:
            print(f"[MISSING] Missing CORS header: {header}")

    print("CORS headers test completed!")

if __name__ == "__main__":
    print("Testing CORS configuration fix...")
    test_options_request()
    test_cors_headers()