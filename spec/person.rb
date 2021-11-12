# frozen_string_literal: true

require "yaml"

# A class that repesents Peson
class Person
  def initialize(hash)
    raise ArgumentError, "Person must be a hash, but got `#{hash.class}`" unless hash.kind_of?(Hash)
    @metadata = hash
    raise ArgumentError, "name is missing: hash: #{hash.inspect}" unless @metadata.key?("name")
  end

  attr_reader :metadata

  def full_name?
    metadata.key?("full_name")
  end

  def full_name
    metadata["full_name"]
  end

  def gh_id?
    metadata.key?("gh_id")
  end

  def gh_id
    metadata["gh_id"]
  end

  def email?
    metadata.key?("email")
  end

  def email
    metadata["email"]
  end

  def website?
    metadata.key?("website")
  end

  def website
    metadata["website"]
  end

  def to_s
    metadata["name"]
  end
end
