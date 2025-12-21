#!/usr/bin/env python3
"""
Detailed test script to verify the RAG API is working properly
"""
import requests
import json

def test_api_detailed():
    base_url = "http://localhost:8000"

    # Test health endpoint
    print("Testing health endpoint...")
    try:
        response = requests.get(f"{base_url}/health")
        print(f"Health check: {response.status_code} - {response.json()}")
    except Exception as e:
        print(f"Health check failed: {e}")

    # Test chat endpoint with more specific query
    print("\nTesting chat endpoint with detailed query...")
    try:
        chat_data = {
            "query": "What is embodied intelligence in Physical AI?",
            "selected_text": None,
            "session_id": "test-session-456"
        }
        response = requests.post(f"{base_url}/chat", json=chat_data, headers={"Content-Type": "application/json"})
        print(f"Chat response status: {response.status_code}")
        if response.status_code == 200:
            result = response.json()
            print(f"Response: '{result.get('response')}'")
            print(f"Full response length: {len(result.get('response', ''))} characters")
            print(f"Sources found: {result.get('sources')}")
            print(f"Session ID: {result.get('session_id')}")
        else:
            print(f"Error response: {response.text}")
    except Exception as e:
        print(f"Chat test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_api_detailed()