# frozen_string_literal: true

require "yaml"

# A class that represents build target

class Target
  def initialize(name)
    raise ArgumentError, "name is missing" unless name

    @name = name
  end

  attr_reader :name
end
