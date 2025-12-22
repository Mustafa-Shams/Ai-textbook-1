import logging
import asyncio
from typing import Optional
from openai import AsyncOpenAI
from config.settings import settings
import random

logger = logging.getLogger(__name__)

class LLMService:
    """
    Service to handle communication with LLM providers via OpenRouter.
    Uses the OpenAI Python client with OpenRouter's API.
    Falls back to rule-based responses when API is unavailable.
    """

    def __init__(self):
        # Check if API key is available
        if settings.OPENROUTER_API_KEY:
            try:
                self.client = AsyncOpenAI(
                    api_key=settings.OPENROUTER_API_KEY,
                    base_url="https://openrouter.ai/api/v1"
                )
                self.model = settings.LLM_MODEL
                self.api_available = True
                logger.info("LLM service initialized with OpenRouter API")
            except Exception as e:
                logger.warning(f"Could not initialize OpenRouter API: {e}")
                self.api_available = False
        else:
            logger.warning("OpenRouter API key not set, using fallback responses")
            self.api_available = False

    async def generate_response(self, query: str, context: str, session_id: Optional[str] = None) -> str:
        """
        Generate response using the LLM with provided context
        """
        try:
            # Handle greetings and simple queries separately - this should always work
            query_lower = query.lower().strip()
            if query_lower in ["hi", "hello", "hey", "greetings", "help", "start", "what can you do", "what are you", "who are you", "introduction"]:
                greeting_message = """Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook. I can help you understand concepts about:

• **ROS 2**: The communication backbone of modern robotics
• **Navigation Systems**: Including the Nav2 stack for path planning
• **Digital Twins**: Simulation and testing environments
• **AI Integration**: Vision-Language-Action systems
• **Humanoid Robotics**: Bipedal locomotion and control

You can ask me about specific topics from the textbook or select text on the page to get detailed explanations.

**Example questions:**
• "What is ROS 2 and why is it important?"
• "Explain the Nav2 stack for bipedal movement"
• "How do humanoid robots maintain balance?"
• "What are the core concepts of ROS 2 communication?"

**To use the highlight feature:** Select text on the page and click the 'Ask AI' tooltip for instant explanations.

I provide accurate, textbook-based information focused on Physical AI and Humanoid Robotics concepts."""
                return greeting_message.strip()

            # If API is not available, use fallback responses based on context
            if not self.api_available:
                return self._generate_fallback_response(query, context)

            # Check if this is a highlighted text explanation request
            if "Explain this:" in query:
                # This is from the highlight feature, use the extracted text as context
                selected_text = query.replace("Explain this:", "").strip()
                system_message = """You are an expert AI assistant for the Physical AI and Humanoid Robotics textbook. Provide a detailed, structured explanation of the selected text with clear sections.

**Selected Text:**
{selected_text}

**Requirements:**
1. Provide a comprehensive breakdown of the concepts
2. Use structured formatting (headings, bullet points, numbered lists)
3. Explain technical terms in simple language
4. Connect concepts to broader Physical AI and Humanoid Robotics principles
5. Include practical applications where relevant
6. Maintain academic rigor while ensuring accessibility
7. Focus on robotics applications and context"""

                messages = [
                    {"role": "system", "content": system_message.format(selected_text=selected_text)},
                    {"role": "user", "content": f"Please provide a detailed, structured explanation of this text: {selected_text}"}
                ]
            else:
                # Regular query with RAG context
                if context.strip():
                    # Check if the context contains relevant information for the query
                    context_lower = context.lower()
                    query_lower = query.lower()

                    # Enhanced relevance checking with more sophisticated matching
                    query_keywords = [word for word in query_lower.split() if len(word) > 2]
                    found_keywords = [kw for kw in query_keywords if kw in context_lower]

                    if len(found_keywords) > 0 or len(query_keywords) == 0:
                        # Context appears to contain relevant information
                        system_message = """You are an expert AI assistant for the Physical AI and Humanoid Robotics textbook. Provide accurate, detailed responses based ONLY on the provided context.

**Context from textbook:**
{context}

**User Question:**
{query}

**Response Guidelines:**
1. Base your answer strictly on the provided context
2. Quote relevant passages when possible
3. Provide structured, well-organized answers
4. Use clear headings and bullet points for complex topics
5. Connect concepts to Physical AI and Humanoid Robotics applications
6. If the context partially answers the question, provide all available information
7. Maintain academic rigor while ensuring clarity
8. Use technical terminology appropriately but explain when needed

**Response Format:**
- Start with a clear, direct answer
- Use structured sections for complex topics
- End with related concepts if relevant"""

                        messages = [
                            {"role": "system", "content": system_message.format(context=context, query=query)},
                            {"role": "user", "content": f"Please answer this question based on the provided context: {query}"}
                        ]
                    else:
                        # Context exists but doesn't seem to directly answer the specific question
                        system_message = """You are an AI assistant for the Physical AI and Humanoid Robotics textbook. The retrieved context contains textbook information but may not directly answer the specific question.

**Retrieved Context:**
{context}

**User Question:**
{query}

**Instructions:**
1. If the context contains related information, extract and present it
2. If the exact question cannot be answered, explain what information is available
3. Suggest related topics that might be helpful
4. Acknowledge limitations honestly
5. Maintain focus on Physical AI and Robotics content
6. Provide the most relevant information possible from the context

**Response Format:**
- Acknowledge the specific question
- Present relevant information from context
- Suggest where to find more specific information if applicable
- Keep focus on robotics and AI concepts"""

                        messages = [
                            {"role": "system", "content": system_message.format(context=context, query=query)},
                            {"role": "user", "content": f"Please answer: '{query}' based on the provided context. If the exact question cannot be answered, explain what relevant information is available."}
                        ]
                else:
                    # No context found, inform the user with helpful suggestions
                    no_context_response = f"""I couldn't find relevant information in the textbook to answer your specific question about "{query}".

**To get better results, try asking about:**
• Specific ROS 2 concepts (nodes, topics, services)
• Navigation systems (Nav2 stack, path planning)
• Digital twin technologies
• AI integration in robotics
• Humanoid robotics principles
• Sensor systems and perception

**You can also:**
• Select text on the page and use the 'Ask AI' feature for instant explanations
• Ask more specific questions about Physical AI and Humanoid Robotics
• Try rephrasing your question to include specific technical terms

Would you like to ask about any of these specific topics instead?"""
                    return no_context_response

            # Call the LLM with enhanced error handling
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.3,  # Lower temperature for more consistent, factual responses
                max_tokens=1200,  # Increased for more detailed responses
                timeout=45.0,     # Increased timeout for complex queries
                top_p=0.9,        # Better diversity control
                frequency_penalty=0.1,  # Reduce repetitive responses
                presence_penalty=0.1    # Encourage topic diversity
            )

            # Extract and return the response
            result = response.choices[0].message.content

            # Post-process the response to ensure quality
            if not result or result.strip() in [" ", "", "  "]:
                return "I understand your question about \"{query}\", but I couldn't generate a response based on the available textbook content. Please try rephrasing your question or select specific text to get an explanation."

            return result

        except Exception as e:
            logger.error(f"Error generating response from LLM: {e}")
            # Use fallback response if there's an error
            return self._generate_fallback_response(query, context)

    def _generate_fallback_response(self, query: str, context: str) -> str:
        """
        Generate a fallback response when API is not available
        """
        # Handle greetings separately (should always work)
        query_lower = query.lower().strip()
        if query_lower in ["hi", "hello", "hey", "greetings"]:
            return """Hello! I'm your AI assistant for the Physical AI and Humanoid Robotics textbook. I'm currently running in offline mode. I can help you understand concepts about:

• **ROS 2**: The communication backbone of modern robotics
• **Navigation Systems**: Including the Nav2 stack for path planning
• **Digital Twins**: Simulation and testing environments
• **AI Integration**: Vision-Language-Action systems
• **Humanoid Robotics**: Bipedal locomotion and control

You can ask me about specific topics from the textbook and I'll do my best to provide helpful information based on the available documentation."""

        # If we have context, try to extract relevant information
        if context and len(context.strip()) > 0:
            # Simple approach: return context with a note
            context_preview = context[:500] + "..." if len(context) > 500 else context
            return f"""I found some relevant information in the textbook:

{context_preview}

This information is from the available documentation. For more detailed explanations, please refer to the specific sections in the textbook."""

        # Default fallback
        return f"""I'm currently running in offline mode without access to the LLM API.
For your query '{query}', I recommend:

• Checking the relevant sections in the Physical AI and Humanoid Robotics textbook
• Looking for information about ROS 2, navigation systems, digital twins, or humanoid robotics
• Using more specific technical terms in your query if available in the documentation

I'm working to provide the best possible response with the available local documentation."""

    async def generate_explanation(self, selected_text: str) -> str:
        """
        Generate an explanation for selected text
        """
        try:
            # If API is not available, use fallback
            if not self.api_available:
                return f"""Selected text explanation (offline mode):\n\n{selected_text}\n\nThis text is from the Physical AI and Humanoid Robotics documentation. For detailed explanations, please refer to the relevant sections in the textbook."""

            system_message = """
            You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Provide a clear, simplified explanation of the following selected text.
            Break down complex concepts into understandable parts.
            """

            messages = [
                {"role": "system", "content": system_message},
                {"role": "user", "content": f"Please explain this text in simple terms:\n\n{selected_text}"}
            ]

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=800,
                timeout=30.0
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating explanation: {e}")
            # Return fallback response
            return f"""Selected text: {selected_text}\n\nCould not generate detailed explanation due to service unavailability. Please refer to the relevant documentation section for detailed information."""

    async def generate_simplified_explanation(self, selected_text: str) -> str:
        """
        Generate a simplified explanation for selected text
        """
        try:
            # If API is not available, use fallback
            if not self.api_available:
                return f"""Simplified explanation (offline mode):\n\nOriginal text: {selected_text}\n\nThis content is from the Physical AI and Humanoid Robotics documentation. The key concept is likely related to robotics, AI, or Physical AI principles."""

            system_message = """
            You are an AI assistant for a Physical AI and Humanoid Robotics textbook.
            Provide a simplified explanation of the following text that is easy to understand.
            Focus on the core concepts and avoid technical jargon where possible.
            """

            messages = [
                {"role": "system", "content": system_message},
                {"role": "user", "content": f"Please simplify this text:\n\n{selected_text}"}
            ]

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=0.7,
                max_tokens=600,
                timeout=30.0
            )

            return response.choices[0].message.content

        except Exception as e:
            logger.error(f"Error generating simplified explanation: {e}")
            # Return fallback response
            return f"""Text to simplify: {selected_text}\n\nCould not generate simplified explanation due to service unavailability. The content appears to be related to Physical AI and Humanoid Robotics concepts."""